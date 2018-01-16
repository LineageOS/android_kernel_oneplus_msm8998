/*
 * BFQ v8r12 for 4.4.0: data structures and common functions prototypes.
 *
 * Based on ideas and code from CFQ:
 * Copyright (C) 2003 Jens Axboe <axboe@kernel.dk>
 *
 * Copyright (C) 2008 Fabio Checconi <fabio@gandalf.sssup.it>
 *		      Paolo Valente <paolo.valente@unimore.it>
 *
 * Copyright (C) 2015 Paolo Valente <paolo.valente@unimore.it>
 *
 * Copyright (C) 2017 Paolo Valente <paolo.valente@linaro.org>
 */

#ifndef _BFQ_H
#define _BFQ_H

#include <linux/blktrace_api.h>
#include <linux/hrtimer.h>
#include <linux/blk-cgroup.h>

#define BFQ_IOPRIO_CLASSES	3
#define BFQ_CL_IDLE_TIMEOUT	(HZ/5)

#define BFQ_MIN_WEIGHT			1
#define BFQ_MAX_WEIGHT			1000
#define BFQ_WEIGHT_CONVERSION_COEFF	10

#define BFQ_DEFAULT_QUEUE_IOPRIO	4

#define BFQ_WEIGHT_LEGACY_DFL	100
#define BFQ_DEFAULT_GRP_IOPRIO	0
#define BFQ_DEFAULT_GRP_CLASS	IOPRIO_CLASS_BE

/*
 * Soft real-time applications are extremely more latency sensitive
 * than interactive ones. Over-raise the weight of the former to
 * privilege them against the latter.
 */
#define BFQ_SOFTRT_WEIGHT_FACTOR	100

struct bfq_entity;

/**
 * struct bfq_service_tree - per ioprio_class service tree.
 *
 * Each service tree represents a B-WF2Q+ scheduler on its own.  Each
 * ioprio_class has its own independent scheduler, and so its own
 * bfq_service_tree.  All the fields are protected by the queue lock
 * of the containing bfqd.
 */
struct bfq_service_tree {
	/* tree for active entities (i.e., those backlogged) */
	struct rb_root active;
	/* tree for idle entities (i.e., not backlogged, with V <= F_i)*/
	struct rb_root idle;

	struct bfq_entity *first_idle;	/* idle entity with minimum F_i */
	struct bfq_entity *last_idle;	/* idle entity with maximum F_i */

	u64 vtime; /* scheduler virtual time */
	/* scheduler weight sum; active and idle entities contribute to it */
	unsigned long wsum;
};

/**
 * struct bfq_sched_data - multi-class scheduler.
 *
 * bfq_sched_data is the basic scheduler queue.  It supports three
 * ioprio_classes, and can be used either as a toplevel queue or as an
 * intermediate queue in a hierarchical setup.
 *
 * The supported ioprio_classes are the same as in CFQ, in descending
 * priority order, IOPRIO_CLASS_RT, IOPRIO_CLASS_BE, IOPRIO_CLASS_IDLE.
 * Requests from higher priority queues are served before all the
 * requests from lower priority queues; among requests of the same
 * queue requests are served according to B-WF2Q+.
 *
 * The schedule is implemented by the service trees, plus the field
 * @next_in_service, which points to the entity on the active trees
 * that will be served next, if 1) no changes in the schedule occurs
 * before the current in-service entity is expired, 2) the in-service
 * queue becomes idle when it expires, and 3) if the entity pointed by
 * in_service_entity is not a queue, then the in-service child entity
 * of the entity pointed by in_service_entity becomes idle on
 * expiration. This peculiar definition allows for the following
 * optimization, not yet exploited: while a given entity is still in
 * service, we already know which is the best candidate for next
 * service among the other active entitities in the same parent
 * entity. We can then quickly compare the timestamps of the
 * in-service entity with those of such best candidate.
 *
 * All the fields are protected by the queue lock of the containing
 * bfqd.
 */
struct bfq_sched_data {
	struct bfq_entity *in_service_entity;  /* entity in service */
	/* head-of-the-line entity in the scheduler (see comments above) */
	struct bfq_entity *next_in_service;
	/* array of service trees, one per ioprio_class */
	struct bfq_service_tree service_tree[BFQ_IOPRIO_CLASSES];
	/* last time CLASS_IDLE was served */
	unsigned long bfq_class_idle_last_service;

};

/**
 * struct bfq_weight_counter - counter of the number of all active entities
 *                             with a given weight.
 */
struct bfq_weight_counter {
	unsigned int weight; /* weight of the entities this counter refers to */
	unsigned int num_active; /* nr of active entities with this weight */
	/*
	 * Weights tree member (see bfq_data's @queue_weights_tree and
	 * @group_weights_tree)
	 */
	struct rb_node weights_node;
};

/**
 * struct bfq_entity - schedulable entity.
 *
 * A bfq_entity is used to represent either a bfq_queue (leaf node in the
 * cgroup hierarchy) or a bfq_group into the upper level scheduler.  Each
 * entity belongs to the sched_data of the parent group in the cgroup
 * hierarchy.  Non-leaf entities have also their own sched_data, stored
 * in @my_sched_data.
 *
 * Each entity stores independently its priority values; this would
 * allow different weights on different devices, but this
 * functionality is not exported to userspace by now.  Priorities and
 * weights are updated lazily, first storing the new values into the
 * new_* fields, then setting the @prio_changed flag.  As soon as
 * there is a transition in the entity state that allows the priority
 * update to take place the effective and the requested priority
 * values are synchronized.
 *
 * Unless cgroups are used, the weight value is calculated from the
 * ioprio to export the same interface as CFQ.  When dealing with
 * ``well-behaved'' queues (i.e., queues that do not spend too much
 * time to consume their budget and have true sequential behavior, and
 * when there are no external factors breaking anticipation) the
 * relative weights at each level of the cgroups hierarchy should be
 * guaranteed.  All the fields are protected by the queue lock of the
 * containing bfqd.
 */
struct bfq_entity {
	struct rb_node rb_node; /* service_tree member */
	/* pointer to the weight counter associated with this entity */
	struct bfq_weight_counter *weight_counter;

	/*
	 * Flag, true if the entity is on a tree (either the active or
	 * the idle one of its service_tree) or is in service.
	 */
	bool on_st;

	u64 finish; /* B-WF2Q+ finish timestamp (aka F_i) */
	u64 start;  /* B-WF2Q+ start timestamp (aka S_i) */

	/* tree the entity is enqueued into; %NULL if not on a tree */
	struct rb_root *tree;

	/*
	 * minimum start time of the (active) subtree rooted at this
	 * entity; used for O(log N) lookups into active trees
	 */
	u64 min_start;

	/* amount of service received during the last service slot */
	int service;

	/* budget, used also to calculate F_i: F_i = S_i + @budget / @weight */
	int budget;

	unsigned int weight;	 /* weight of the queue */
	unsigned int new_weight; /* next weight if a change is in progress */

	/* original weight, used to implement weight boosting */
	unsigned int orig_weight;

	/* parent entity, for hierarchical scheduling */
	struct bfq_entity *parent;

	/*
	 * For non-leaf nodes in the hierarchy, the associated
	 * scheduler queue, %NULL on leaf nodes.
	 */
	struct bfq_sched_data *my_sched_data;
	/* the scheduler queue this entity belongs to */
	struct bfq_sched_data *sched_data;

	/* flag, set to request a weight, ioprio or ioprio_class change  */
	int prio_changed;
};

struct bfq_group;

/**
 * struct bfq_queue - leaf schedulable entity.
 *
 * A bfq_queue is a leaf request queue; it can be associated with an
 * io_context or more, if it  is  async or shared  between  cooperating
 * processes. @cgroup holds a reference to the cgroup, to be sure that it
 * does not disappear while a bfqq still references it (mostly to avoid
 * races between request issuing and task migration followed by cgroup
 * destruction).
 * All the fields are protected by the queue lock of the containing bfqd.
 */
struct bfq_queue {
	/* reference counter */
	int ref;
	/* parent bfq_data */
	struct bfq_data *bfqd;

	/* current ioprio and ioprio class */
	unsigned short ioprio, ioprio_class;
	/* next ioprio and ioprio class if a change is in progress */
	unsigned short new_ioprio, new_ioprio_class;

	/*
	 * Shared bfq_queue if queue is cooperating with one or more
	 * other queues.
	 */
	struct bfq_queue *new_bfqq;
	/* request-position tree member (see bfq_group's @rq_pos_tree) */
	struct rb_node pos_node;
	/* request-position tree root (see bfq_group's @rq_pos_tree) */
	struct rb_root *pos_root;

	/* sorted list of pending requests */
	struct rb_root sort_list;
	/* if fifo isn't expired, next request to serve */
	struct request *next_rq;
	/* number of sync and async requests queued */
	int queued[2];
	/* number of sync and async requests currently allocated */
	int allocated[2];
	/* number of pending metadata requests */
	int meta_pending;
	/* fifo list of requests in sort_list */
	struct list_head fifo;

	/* entity representing this queue in the scheduler */
	struct bfq_entity entity;

	/* maximum budget allowed from the feedback mechanism */
	int max_budget;
	/* budget expiration (in jiffies) */
	unsigned long budget_timeout;

	/* number of requests on the dispatch list or inside driver */
	int dispatched;

	unsigned int flags; /* status flags.*/

	/* node for active/idle bfqq list inside parent bfqd */
	struct list_head bfqq_list;

	/* bit vector: a 1 for each seeky requests in history */
	u32 seek_history;

	/* node for the device's burst list */
	struct hlist_node burst_list_node;

	/* position of the last request enqueued */
	sector_t last_request_pos;

	/* Number of consecutive pairs of request completion and
	 * arrival, such that the queue becomes idle after the
	 * completion, but the next request arrives within an idle
	 * time slice; used only if the queue's IO_bound flag has been
	 * cleared.
	 */
	unsigned int requests_within_timer;

	/* pid of the process owning the queue, used for logging purposes */
	pid_t pid;

	/*
	 * Pointer to the bfq_io_cq owning the bfq_queue, set to %NULL
	 * if the queue is shared.
	 */
	struct bfq_io_cq *bic;

	/* current maximum weight-raising time for this queue */
	unsigned long wr_cur_max_time;
	/*
	 * Minimum time instant such that, only if a new request is
	 * enqueued after this time instant in an idle @bfq_queue with
	 * no outstanding requests, then the task associated with the
	 * queue it is deemed as soft real-time (see the comments on
	 * the function bfq_bfqq_softrt_next_start())
	 */
	unsigned long soft_rt_next_start;
	/*
	 * Start time of the current weight-raising period if
	 * the @bfq-queue is being weight-raised, otherwise
	 * finish time of the last weight-raising period.
	 */
	unsigned long last_wr_start_finish;
	/* factor by which the weight of this queue is multiplied */
	unsigned int wr_coeff;
	/*
	 * Time of the last transition of the @bfq_queue from idle to
	 * backlogged.
	 */
	unsigned long last_idle_bklogged;
	/*
	 * Cumulative service received from the @bfq_queue since the
	 * last transition from idle to backlogged.
	 */
	unsigned long service_from_backlogged;
	/*
	 * Value of wr start time when switching to soft rt
	 */
	unsigned long wr_start_at_switch_to_srt;

	unsigned long split_time; /* time of last split */
};

/**
 * struct bfq_ttime - per process thinktime stats.
 */
struct bfq_ttime {
	u64 last_end_request; /* completion time of last request */

	u64 ttime_total; /* total process thinktime */
	unsigned long ttime_samples; /* number of thinktime samples */
	u64 ttime_mean; /* average process thinktime */

};

/**
 * struct bfq_io_cq - per (request_queue, io_context) structure.
 */
struct bfq_io_cq {
	/* associated io_cq structure */
	struct io_cq icq; /* must be the first member */
	/* array of two process queues, the sync and the async */
	struct bfq_queue *bfqq[2];
	/* associated @bfq_ttime struct */
	struct bfq_ttime ttime;
	/* per (request_queue, blkcg) ioprio */
	int ioprio;
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	uint64_t blkcg_serial_nr; /* the current blkcg serial */
#endif

	/*
	 * Snapshot of the has_short_time flag before merging; taken
	 * to remember its value while the queue is merged, so as to
	 * be able to restore it in case of split.
	 */
	bool saved_has_short_ttime;
	/*
	 * Same purpose as the previous two fields for the I/O bound
	 * classification of a queue.
	 */
	bool saved_IO_bound;

	/*
	 * Same purpose as the previous fields for the value of the
	 * field keeping the queue's belonging to a large burst
	 */
	bool saved_in_large_burst;
	/*
	 * True if the queue belonged to a burst list before its merge
	 * with another cooperating queue.
	 */
	bool was_in_burst_list;

	/*
	 * Similar to previous fields: save wr information.
	 */
	unsigned long saved_wr_coeff;
	unsigned long saved_last_wr_start_finish;
	unsigned long saved_wr_start_at_switch_to_srt;
	unsigned int saved_wr_cur_max_time;
};

enum bfq_device_speed {
	BFQ_BFQD_FAST,
	BFQ_BFQD_SLOW,
};

/**
 * struct bfq_data - per-device data structure.
 *
 * All the fields are protected by the @queue lock.
 */
struct bfq_data {
	/* request queue for the device */
	struct request_queue *queue;

	/* root bfq_group for the device */
	struct bfq_group *root_group;

	/*
	 * rbtree of weight counters of @bfq_queues, sorted by
	 * weight. Used to keep track of whether all @bfq_queues have
	 * the same weight. The tree contains one counter for each
	 * distinct weight associated to some active and not
	 * weight-raised @bfq_queue (see the comments to the functions
	 * bfq_weights_tree_[add|remove] for further details).
	 */
	struct rb_root queue_weights_tree;
	/*
	 * rbtree of non-queue @bfq_entity weight counters, sorted by
	 * weight. Used to keep track of whether all @bfq_groups have
	 * the same weight. The tree contains one counter for each
	 * distinct weight associated to some active @bfq_group (see
	 * the comments to the functions bfq_weights_tree_[add|remove]
	 * for further details).
	 */
	struct rb_root group_weights_tree;

	/*
	 * Number of bfq_queues containing requests (including the
	 * queue in service, even if it is idling).
	 */
	int busy_queues;
	/* number of weight-raised busy @bfq_queues */
	int wr_busy_queues;
	/* number of queued requests */
	int queued;
	/* number of requests dispatched and waiting for completion */
	int rq_in_driver;

	/*
	 * Maximum number of requests in driver in the last
	 * @hw_tag_samples completed requests.
	 */
	int max_rq_in_driver;
	/* number of samples used to calculate hw_tag */
	int hw_tag_samples;
	/* flag set to one if the driver is showing a queueing behavior */
	int hw_tag;

	/* number of budgets assigned */
	int budgets_assigned;

	/*
	 * Timer set when idling (waiting) for the next request from
	 * the queue in service.
	 */
	struct hrtimer idle_slice_timer;
	/* delayed work to restart dispatching on the request queue */
	struct work_struct unplug_work;

	/* bfq_queue in service */
	struct bfq_queue *in_service_queue;
	/* bfq_io_cq (bic) associated with the @in_service_queue */
	struct bfq_io_cq *in_service_bic;

	/* on-disk position of the last served request */
	sector_t last_position;

	/* time of last request completion (ns) */
	u64 last_completion;

	/* time of first rq dispatch in current observation interval (ns) */
	u64 first_dispatch;
	/* time of last rq dispatch in current observation interval (ns) */
	u64 last_dispatch;

	/* beginning of the last budget */
	ktime_t last_budget_start;
	/* beginning of the last idle slice */
	ktime_t last_idling_start;

	/* number of samples in current observation interval */
	int peak_rate_samples;
	/* num of samples of seq dispatches in current observation interval */
	u32 sequential_samples;
	/* total num of sectors transferred in current observation interval */
	u64 tot_sectors_dispatched;
	/* max rq size seen during current observation interval (sectors) */
	u32 last_rq_max_size;
	/* time elapsed from first dispatch in current observ. interval (us) */
	u64 delta_from_first;
	/* current estimate of device peak rate */
	u32 peak_rate;

	/* maximum budget allotted to a bfq_queue before rescheduling */
	int bfq_max_budget;

	/* list of all the bfq_queues active on the device */
	struct list_head active_list;
	/* list of all the bfq_queues idle on the device */
	struct list_head idle_list;

	/*
	 * Timeout for async/sync requests; when it fires, requests
	 * are served in fifo order.
	 */
	u64 bfq_fifo_expire[2];
	/* weight of backward seeks wrt forward ones */
	unsigned int bfq_back_penalty;
	/* maximum allowed backward seek */
	unsigned int bfq_back_max;
	/* maximum idling time */
	u32 bfq_slice_idle;

	/* user-configured max budget value (0 for auto-tuning) */
	int bfq_user_max_budget;
	/*
	 * Timeout for bfq_queues to consume their budget; used to
	 * prevent seeky queues from imposing long latencies to
	 * sequential or quasi-sequential ones (this also implies that
	 * seeky queues cannot receive guarantees in the service
	 * domain; after a timeout they are charged for the time they
	 * have been in service, to preserve fairness among them, but
	 * without service-domain guarantees).
	 */
	unsigned int bfq_timeout;

	/*
	 * Number of consecutive requests that must be issued within
	 * the idle time slice to set again idling to a queue which
	 * was marked as non-I/O-bound (see the definition of the
	 * IO_bound flag for further details).
	 */
	unsigned int bfq_requests_within_timer;

	/*
	 * Force device idling whenever needed to provide accurate
	 * service guarantees, without caring about throughput
	 * issues. CAVEAT: this may even increase latencies, in case
	 * of useless idling for processes that did stop doing I/O.
	 */
	bool strict_guarantees;

	/*
	 * Last time at which a queue entered the current burst of
	 * queues being activated shortly after each other; for more
	 * details about this and the following parameters related to
	 * a burst of activations, see the comments on the function
	 * bfq_handle_burst.
	 */
	unsigned long last_ins_in_burst;
	/*
	 * Reference time interval used to decide whether a queue has
	 * been activated shortly after @last_ins_in_burst.
	 */
	unsigned long bfq_burst_interval;
	/* number of queues in the current burst of queue activations */
	int burst_size;

	/* common parent entity for the queues in the burst */
	struct bfq_entity *burst_parent_entity;
	/* Maximum burst size above which the current queue-activation
	 * burst is deemed as 'large'.
	 */
	unsigned long bfq_large_burst_thresh;
	/* true if a large queue-activation burst is in progress */
	bool large_burst;
	/*
	 * Head of the burst list (as for the above fields, more
	 * details in the comments on the function bfq_handle_burst).
	 */
	struct hlist_head burst_list;

	/* if set to true, low-latency heuristics are enabled */
	bool low_latency;
	/*
	 * Maximum factor by which the weight of a weight-raised queue
	 * is multiplied.
	 */
	unsigned int bfq_wr_coeff;
	/* maximum duration of a weight-raising period (jiffies) */
	unsigned int bfq_wr_max_time;

	/* Maximum weight-raising duration for soft real-time processes */
	unsigned int bfq_wr_rt_max_time;
	/*
	 * Minimum idle period after which weight-raising may be
	 * reactivated for a queue (in jiffies).
	 */
	unsigned int bfq_wr_min_idle_time;
	/*
	 * Minimum period between request arrivals after which
	 * weight-raising may be reactivated for an already busy async
	 * queue (in jiffies).
	 */
	unsigned long bfq_wr_min_inter_arr_async;

	/* Max service-rate for a soft real-time queue, in sectors/sec */
	unsigned int bfq_wr_max_softrt_rate;
	/*
	 * Cached value of the product R*T, used for computing the
	 * maximum duration of weight raising automatically.
	 */
	u64 RT_prod;
	/* device-speed class for the low-latency heuristic */
	enum bfq_device_speed device_speed;

	/* fallback dummy bfqq for extreme OOM conditions */
	struct bfq_queue oom_bfqq;
};

enum bfqq_state_flags {
	BFQ_BFQQ_FLAG_just_created = 0,	/* queue just allocated */
	BFQ_BFQQ_FLAG_busy,		/* has requests or is in service */
	BFQ_BFQQ_FLAG_wait_request,	/* waiting for a request */
	BFQ_BFQQ_FLAG_non_blocking_wait_rq, /*
					     * waiting for a request
					     * without idling the device
					     */
	BFQ_BFQQ_FLAG_must_alloc,	/* must be allowed rq alloc */
	BFQ_BFQQ_FLAG_fifo_expire,	/* FIFO checked in this slice */
	BFQ_BFQQ_FLAG_has_short_ttime,	/* queue has a short think time */
	BFQ_BFQQ_FLAG_sync,		/* synchronous queue */
	BFQ_BFQQ_FLAG_IO_bound,		/*
					 * bfqq has timed-out at least once
					 * having consumed at most 2/10 of
					 * its budget
					 */
	BFQ_BFQQ_FLAG_in_large_burst,	/*
					 * bfqq activated in a large burst,
					 * see comments to bfq_handle_burst.
					 */
	BFQ_BFQQ_FLAG_softrt_update,	/*
					 * may need softrt-next-start
					 * update
					 */
	BFQ_BFQQ_FLAG_coop,		/* bfqq is shared */
	BFQ_BFQQ_FLAG_split_coop	/* shared bfqq will be split */
};

#define BFQ_BFQQ_FNS(name)						\
static void bfq_mark_bfqq_##name(struct bfq_queue *bfqq)		\
{									\
	(bfqq)->flags |= (1 << BFQ_BFQQ_FLAG_##name);			\
}									\
static void bfq_clear_bfqq_##name(struct bfq_queue *bfqq)		\
{									\
	(bfqq)->flags &= ~(1 << BFQ_BFQQ_FLAG_##name);			\
}									\
static int bfq_bfqq_##name(const struct bfq_queue *bfqq)		\
{									\
	return ((bfqq)->flags & (1 << BFQ_BFQQ_FLAG_##name)) != 0;	\
}

BFQ_BFQQ_FNS(just_created);
BFQ_BFQQ_FNS(busy);
BFQ_BFQQ_FNS(wait_request);
BFQ_BFQQ_FNS(non_blocking_wait_rq);
BFQ_BFQQ_FNS(must_alloc);
BFQ_BFQQ_FNS(fifo_expire);
BFQ_BFQQ_FNS(has_short_ttime);
BFQ_BFQQ_FNS(sync);
BFQ_BFQQ_FNS(IO_bound);
BFQ_BFQQ_FNS(in_large_burst);
BFQ_BFQQ_FNS(coop);
BFQ_BFQQ_FNS(split_coop);
BFQ_BFQQ_FNS(softrt_update);
#undef BFQ_BFQQ_FNS

/* Logging facilities. */
#ifdef CONFIG_BFQ_REDIRECT_TO_CONSOLE
#ifdef CONFIG_BFQ_GROUP_IOSCHED
static struct bfq_group *bfqq_group(struct bfq_queue *bfqq);
static struct blkcg_gq *bfqg_to_blkg(struct bfq_group *bfqg);

#define bfq_log_bfqq(bfqd, bfqq, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	assert_spin_locked((bfqd)->queue->queue_lock);			\
	blkg_path(bfqg_to_blkg(bfqq_group(bfqq)), __pbuf, sizeof(__pbuf)); \
	pr_crit("bfq%d%c %s " fmt "\n", 			\
		(bfqq)->pid,						\
		bfq_bfqq_sync((bfqq)) ? 'S' : 'A',			\
		__pbuf, ##args);					\
} while (0)

#define bfq_log_bfqg(bfqd, bfqg, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(bfqg_to_blkg(bfqg), __pbuf, sizeof(__pbuf));		\
	pr_crit("%s " fmt "\n", __pbuf, ##args);	\
} while (0)

#else /* CONFIG_BFQ_GROUP_IOSCHED */

#define bfq_log_bfqq(bfqd, bfqq, fmt, args...)		\
	pr_crit("bfq%d%c " fmt "\n", (bfqq)->pid,		\
		bfq_bfqq_sync((bfqq)) ? 'S' : 'A',	\
		##args)
#define bfq_log_bfqg(bfqd, bfqg, fmt, args...)		do {} while (0)

#endif /* CONFIG_BFQ_GROUP_IOSCHED */

#define bfq_log(bfqd, fmt, args...) \
	pr_crit("bfq " fmt "\n", ##args)

#else /* CONFIG_BFQ_REDIRECT_TO_CONSOLE */
#ifdef CONFIG_BFQ_GROUP_IOSCHED
static struct bfq_group *bfqq_group(struct bfq_queue *bfqq);
static struct blkcg_gq *bfqg_to_blkg(struct bfq_group *bfqg);

#define bfq_log_bfqq(bfqd, bfqq, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	assert_spin_locked((bfqd)->queue->queue_lock);			\
	blkg_path(bfqg_to_blkg(bfqq_group(bfqq)), __pbuf, sizeof(__pbuf)); \
	blk_add_trace_msg((bfqd)->queue, "bfq%d%c %s " fmt, \
			  (bfqq)->pid,			  \
			  bfq_bfqq_sync((bfqq)) ? 'S' : 'A',	\
			  __pbuf, ##args);				\
} while (0)

#define bfq_log_bfqg(bfqd, bfqg, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(bfqg_to_blkg(bfqg), __pbuf, sizeof(__pbuf));		\
	blk_add_trace_msg((bfqd)->queue, "%s " fmt, __pbuf, ##args);	\
} while (0)

#else /* CONFIG_BFQ_GROUP_IOSCHED */

#define bfq_log_bfqq(bfqd, bfqq, fmt, args...)	\
	blk_add_trace_msg((bfqd)->queue, "bfq%d%c " fmt, (bfqq)->pid,	\
			bfq_bfqq_sync((bfqq)) ? 'S' : 'A',		\
				##args)
#define bfq_log_bfqg(bfqd, bfqg, fmt, args...)		do {} while (0)

#endif /* CONFIG_BFQ_GROUP_IOSCHED */

#define bfq_log(bfqd, fmt, args...) \
	blk_add_trace_msg((bfqd)->queue, "bfq " fmt, ##args)
#endif /* CONFIG_BFQ_REDIRECT_TO_CONSOLE */

/* Expiration reasons. */
enum bfqq_expiration {
	BFQ_BFQQ_TOO_IDLE = 0,		/*
					 * queue has been idling for
					 * too long
					 */
	BFQ_BFQQ_BUDGET_TIMEOUT,	/* budget took too long to be used */
	BFQ_BFQQ_BUDGET_EXHAUSTED,	/* budget consumed */
	BFQ_BFQQ_NO_MORE_REQUESTS,	/* the queue has no more requests */
	BFQ_BFQQ_PREEMPTED		/* preemption in progress */
};


struct bfqg_stats {
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	/* number of ios merged */
	struct blkg_rwstat		merged;
	/* total time spent on device in ns, may not be accurate w/ queueing */
	struct blkg_rwstat		service_time;
	/* total time spent waiting in scheduler queue in ns */
	struct blkg_rwstat		wait_time;
	/* number of IOs queued up */
	struct blkg_rwstat		queued;
	/* total disk time and nr sectors dispatched by this group */
	struct blkg_stat		time;
	/* sum of number of ios queued across all samples */
	struct blkg_stat		avg_queue_size_sum;
	/* count of samples taken for average */
	struct blkg_stat		avg_queue_size_samples;
	/* how many times this group has been removed from service tree */
	struct blkg_stat		dequeue;
	/* total time spent waiting for it to be assigned a timeslice. */
	struct blkg_stat		group_wait_time;
	/* time spent idling for this blkcg_gq */
	struct blkg_stat		idle_time;
	/* total time with empty current active q with other requests queued */
	struct blkg_stat		empty_time;
	/* fields after this shouldn't be cleared on stat reset */
	uint64_t			start_group_wait_time;
	uint64_t			start_idle_time;
	uint64_t			start_empty_time;
	uint16_t			flags;
#endif
};

#ifdef CONFIG_BFQ_GROUP_IOSCHED
/*
 * struct bfq_group_data - per-blkcg storage for the blkio subsystem.
 *
 * @ps: @blkcg_policy_storage that this structure inherits
 * @weight: weight of the bfq_group
 */
struct bfq_group_data {
	/* must be the first member */
	struct blkcg_policy_data pd;

	unsigned int weight;
};

/**
 * struct bfq_group - per (device, cgroup) data structure.
 * @entity: schedulable entity to insert into the parent group sched_data.
 * @sched_data: own sched_data, to contain child entities (they may be
 *              both bfq_queues and bfq_groups).
 * @bfqd: the bfq_data for the device this group acts upon.
 * @async_bfqq: array of async queues for all the tasks belonging to
 *              the group, one queue per ioprio value per ioprio_class,
 *              except for the idle class that has only one queue.
 * @async_idle_bfqq: async queue for the idle class (ioprio is ignored).
 * @my_entity: pointer to @entity, %NULL for the toplevel group; used
 *             to avoid too many special cases during group creation/
 *             migration.
 * @active_entities: number of active entities belonging to the group;
 *                   unused for the root group. Used to know whether there
 *                   are groups with more than one active @bfq_entity
 *                   (see the comments to the function
 *                   bfq_bfqq_may_idle()).
 * @rq_pos_tree: rbtree sorted by next_request position, used when
 *               determining if two or more queues have interleaving
 *               requests (see bfq_find_close_cooperator()).
 *
 * Each (device, cgroup) pair has its own bfq_group, i.e., for each cgroup
 * there is a set of bfq_groups, each one collecting the lower-level
 * entities belonging to the group that are acting on the same device.
 *
 * Locking works as follows:
 *    o @bfqd is protected by the queue lock, RCU is used to access it
 *      from the readers.
 *    o All the other fields are protected by the @bfqd queue lock.
 */
struct bfq_group {
	/* must be the first member */
	struct blkg_policy_data pd;

	struct bfq_entity entity;
	struct bfq_sched_data sched_data;

	void *bfqd;

	struct bfq_queue *async_bfqq[2][IOPRIO_BE_NR];
	struct bfq_queue *async_idle_bfqq;

	struct bfq_entity *my_entity;

	int active_entities;

	struct rb_root rq_pos_tree;

	struct bfqg_stats stats;
};

#else
struct bfq_group {
	struct bfq_sched_data sched_data;

	struct bfq_queue *async_bfqq[2][IOPRIO_BE_NR];
	struct bfq_queue *async_idle_bfqq;

	struct rb_root rq_pos_tree;
};
#endif

static struct bfq_queue *bfq_entity_to_bfqq(struct bfq_entity *entity);

static unsigned int bfq_class_idx(struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);

	return bfqq ? bfqq->ioprio_class - 1 :
		BFQ_DEFAULT_GRP_CLASS - 1;
}

static struct bfq_service_tree *
bfq_entity_service_tree(struct bfq_entity *entity)
{
	struct bfq_sched_data *sched_data = entity->sched_data;
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
	unsigned int idx = bfq_class_idx(entity);

	BUG_ON(idx >= BFQ_IOPRIO_CLASSES);
	BUG_ON(sched_data == NULL);

	if (bfqq)
		bfq_log_bfqq(bfqq->bfqd, bfqq,
			     "entity_service_tree %p %d",
			     sched_data->service_tree + idx, idx);
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	else {
		struct bfq_group *bfqg =
			container_of(entity, struct bfq_group, entity);

		bfq_log_bfqg((struct bfq_data *)bfqg->bfqd, bfqg,
			     "entity_service_tree %p %d",
			     sched_data->service_tree + idx, idx);
	}
#endif
	return sched_data->service_tree + idx;
}

static struct bfq_queue *bic_to_bfqq(struct bfq_io_cq *bic, bool is_sync)
{
	return bic->bfqq[is_sync];
}

static void bic_set_bfqq(struct bfq_io_cq *bic, struct bfq_queue *bfqq,
			 bool is_sync)
{
	bic->bfqq[is_sync] = bfqq;
}

static struct bfq_data *bic_to_bfqd(struct bfq_io_cq *bic)
{
	return bic->icq.q->elevator->elevator_data;
}

#ifdef CONFIG_BFQ_GROUP_IOSCHED

static struct bfq_group *bfq_bfqq_to_bfqg(struct bfq_queue *bfqq)
{
	struct bfq_entity *group_entity = bfqq->entity.parent;

	if (!group_entity)
		group_entity = &bfqq->bfqd->root_group->entity;

	return container_of(group_entity, struct bfq_group, entity);
}

#else

static struct bfq_group *bfq_bfqq_to_bfqg(struct bfq_queue *bfqq)
{
	return bfqq->bfqd->root_group;
}

#endif

static void bfq_check_ioprio_change(struct bfq_io_cq *bic, struct bio *bio);
static void bfq_put_queue(struct bfq_queue *bfqq);
static void bfq_dispatch_insert(struct request_queue *q, struct request *rq);
static struct bfq_queue *bfq_get_queue(struct bfq_data *bfqd,
				       struct bio *bio, bool is_sync,
				       struct bfq_io_cq *bic);
static void bfq_end_wr_async_queues(struct bfq_data *bfqd,
				    struct bfq_group *bfqg);
#ifdef CONFIG_BFQ_GROUP_IOSCHED
static void bfq_put_async_queues(struct bfq_data *bfqd, struct bfq_group *bfqg);
#endif
static void bfq_exit_bfqq(struct bfq_data *bfqd, struct bfq_queue *bfqq);

#endif /* _BFQ_H */
