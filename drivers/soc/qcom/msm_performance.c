// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/moduleparam.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <trace/events/power.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/sched/core_ctl.h>

/*
 * Sched will provide the data for every 20ms window,
 * will collect the data for 15 windows(300ms) and then update
 * sysfs nodes with aggregated data
 */
#define POLL_INT 25
#define NODE_NAME_MAX_CHARS 16
static unsigned int use_input_evts_with_hi_slvt_detect;
static struct mutex managed_cpus_lock;
static int touchboost = 0;

/* Maximum number to clusters that this module will manage*/
static unsigned int num_clusters;
struct cluster {
	cpumask_var_t cpus;
	/* Number of CPUs to maintain online */
	int max_cpu_request;
	/* To track CPUs that the module decides to offline */
	cpumask_var_t offlined_cpus;
	/* stats for load detection */
	/* IO */
	u64 last_io_check_ts;
	unsigned int iowait_enter_cycle_cnt;
	unsigned int iowait_exit_cycle_cnt;
	spinlock_t iowait_lock;
	unsigned int cur_io_busy;
	bool io_change;
	/* CPU */
	unsigned int mode;
	bool mode_change;
	u64 last_mode_check_ts;
	unsigned int single_enter_cycle_cnt;
	unsigned int single_exit_cycle_cnt;
	unsigned int multi_enter_cycle_cnt;
	unsigned int multi_exit_cycle_cnt;
	spinlock_t mode_lock;
	/* Perf Cluster Peak Loads */
	unsigned int perf_cl_peak;
	u64 last_perf_cl_check_ts;
	bool perf_cl_detect_state_change;
	unsigned int perf_cl_peak_enter_cycle_cnt;
	unsigned int perf_cl_peak_exit_cycle_cnt;
	spinlock_t perf_cl_peak_lock;
	/* Tunables */
	unsigned int single_enter_load;
	unsigned int pcpu_multi_enter_load;
	unsigned int perf_cl_peak_enter_load;
	unsigned int single_exit_load;
	unsigned int pcpu_multi_exit_load;
	unsigned int perf_cl_peak_exit_load;
	unsigned int single_enter_cycles;
	unsigned int single_exit_cycles;
	unsigned int multi_enter_cycles;
	unsigned int multi_exit_cycles;
	unsigned int perf_cl_peak_enter_cycles;
	unsigned int perf_cl_peak_exit_cycles;
	unsigned int current_freq;
	spinlock_t timer_lock;
	unsigned int timer_rate;
	struct timer_list mode_exit_timer;
	struct timer_list perf_cl_peak_mode_exit_timer;
};

struct input_events {
	unsigned int evt_x_cnt;
	unsigned int evt_y_cnt;
	unsigned int evt_pres_cnt;
	unsigned int evt_dist_cnt;
};

enum cpu_clusters {
	MIN = 0,
	MID = 1,
	MAX = 2,
	CLUSTER_MAX
};

/* To handle cpufreq min/max request */
struct cpu_status {
	unsigned int min;
	unsigned int max;
};
static DEFINE_PER_CPU(struct cpu_status, msm_perf_cpu_stats);

struct events {
	spinlock_t cpu_hotplug_lock;
	bool cpu_hotplug;
	bool init_success;
};
static struct events events_group;
static struct task_struct *events_notify_thread;

static unsigned int aggr_big_nr;
static unsigned int aggr_top_load;
static unsigned int top_load[CLUSTER_MAX];
static unsigned int curr_cap[CLUSTER_MAX];
#define LAST_UPDATE_TOL		USEC_PER_MSEC

/* Bitmask to keep track of the workloads being detected */
static unsigned int workload_detect;
#define IO_DETECT	1
#define MODE_DETECT	2
#define PERF_CL_PEAK_DETECT	4


/* IOwait related tunables */
static unsigned int io_enter_cycles = 4;
static unsigned int io_exit_cycles = 4;
static u64 iowait_ceiling_pct = 25;
static u64 iowait_floor_pct = 8;
#define LAST_IO_CHECK_TOL	(3 * USEC_PER_MSEC)

static unsigned int aggr_iobusy;
static unsigned int aggr_mode;

static struct task_struct *notify_thread;

static struct input_handler *handler;

/* CPU workload detection related */
#define NO_MODE		(0)
#define SINGLE		(1)
#define MULTI		(2)
#define MIXED		(3)
#define PERF_CL_PEAK		(4)
#define DEF_SINGLE_ENT		90
#define DEF_PCPU_MULTI_ENT	85
#define DEF_PERF_CL_PEAK_ENT	80
#define DEF_SINGLE_EX		60
#define DEF_PCPU_MULTI_EX	50
#define DEF_PERF_CL_PEAK_EX		70
#define DEF_SINGLE_ENTER_CYCLE	4
#define DEF_SINGLE_EXIT_CYCLE	4
#define DEF_MULTI_ENTER_CYCLE	4
#define DEF_MULTI_EXIT_CYCLE	4
#define DEF_PERF_CL_PEAK_ENTER_CYCLE	100
#define DEF_PERF_CL_PEAK_EXIT_CYCLE	20
#define LAST_LD_CHECK_TOL	(2 * USEC_PER_MSEC)
#define CLUSTER_0_THRESHOLD_FREQ	147000
#define CLUSTER_1_THRESHOLD_FREQ	190000
#define INPUT_EVENT_CNT_THRESHOLD	15
#define MAX_LENGTH_CPU_STRING	256



/**************************sysfs start********************************/

static int set_touchboost(const char *buf, const struct kernel_param *kp)
 {
 	int val;

 	if (sscanf(buf, "%d\n", &val) != 1)
 		return -EINVAL;

 	touchboost = val;

 	return 0;
 }

 static int get_touchboost(char *buf, const struct kernel_param *kp)
 {
 	return snprintf(buf, PAGE_SIZE, "%d", touchboost);
 }

 static const struct kernel_param_ops param_ops_touchboost = {
 	.set = set_touchboost,
 	.get = get_touchboost,
 };
 device_param_cb(touchboost, &param_ops_touchboost, NULL, 0644);

static int set_num_clusters(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	if (num_clusters)
		return -EINVAL;

	num_clusters = val;

	if (init_cluster_control()) {
		num_clusters = 0;
		return -ENOMEM;
	}

	return 0;
}

static int get_num_clusters(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", num_clusters);
}

static const struct kernel_param_ops param_ops_num_clusters = {
	.set = set_num_clusters,
	.get = get_num_clusters,
};
device_param_cb(num_clusters, &param_ops_num_clusters, NULL, 0644);

static int set_max_cpus(const char *buf, const struct kernel_param *kp)
{
	unsigned int i, ntokens = 0;
	const char *cp = buf;
	int val;

	if (!clusters_inited)
		return -EINVAL;

	while ((cp = strpbrk(cp + 1, ":")))
		ntokens++;

	if (ntokens != (num_clusters - 1))
		return -EINVAL;

	cp = buf;
	for (i = 0; i < num_clusters; i++) {

		if (sscanf(cp, "%d\n", &val) != 1)
			return -EINVAL;
		if (val > (int)cpumask_weight(managed_clusters[i]->cpus))
			return -EINVAL;

		managed_clusters[i]->max_cpu_request = val;

		cp = strnchr(cp, strlen(cp), ':');
		cp++;
		trace_set_max_cpus(cpumask_bits(managed_clusters[i]->cpus)[0],
								val);
	}

	schedule_delayed_work(&evaluate_hotplug_work, 0);

	return 0;
}

static int get_max_cpus(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:", managed_clusters[i]->max_cpu_request);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_max_cpus = {
	.set = set_max_cpus,
	.get = get_max_cpus,
};

#ifdef CONFIG_MSM_PERFORMANCE_HOTPLUG_ON
device_param_cb(max_cpus, &param_ops_max_cpus, NULL, 0644);
#endif

static int set_managed_cpus(const char *buf, const struct kernel_param *kp)
{
	int i, ret;
	struct cpumask tmp_mask;

	if (!clusters_inited)
		return -EINVAL;

	ret = cpulist_parse(buf, &tmp_mask);

	if (ret)
		return ret;

	for (i = 0; i < num_clusters; i++) {
		if (cpumask_empty(managed_clusters[i]->cpus)) {
			mutex_lock(&managed_cpus_lock);
			cpumask_copy(managed_clusters[i]->cpus, &tmp_mask);
			cpumask_clear(managed_clusters[i]->offlined_cpus);
			mutex_unlock(&managed_cpus_lock);
			break;
		}
	}

	return ret;
}

static int get_managed_cpus(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0, total_cnt = 0;
	char tmp[MAX_LENGTH_CPU_STRING] = "";

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++) {
		cnt = cpumap_print_to_pagebuf(true, buf,
						managed_clusters[i]->cpus);
		if ((i + 1) < num_clusters &&
		    (total_cnt + cnt + 1) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			tmp[cnt-1] = ':';
			tmp[cnt] = '\0';
			total_cnt += cnt;
		} else if ((i + 1) == num_clusters &&
			   (total_cnt + cnt) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			total_cnt += cnt;
		} else {
			pr_err("invalid string for managed_cpu:%s%s\n", tmp,
				buf);
			break;
		}
	}
	snprintf(buf, PAGE_SIZE, "%s", tmp);
	return total_cnt;
}

static const struct kernel_param_ops param_ops_managed_cpus = {
	.set = set_managed_cpus,
	.get = get_managed_cpus,
};
device_param_cb(managed_cpus, &param_ops_managed_cpus, NULL, 0644);

/* Read-only node: To display all the online managed CPUs */
static int get_managed_online_cpus(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0, total_cnt = 0;
	char tmp[MAX_LENGTH_CPU_STRING] = "";
	struct cpumask tmp_mask;
	struct cluster *i_cl;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++) {
		i_cl = managed_clusters[i];

		cpumask_clear(&tmp_mask);
		cpumask_complement(&tmp_mask, i_cl->offlined_cpus);
		cpumask_and(&tmp_mask, i_cl->cpus, &tmp_mask);

		cnt = cpumap_print_to_pagebuf(true, buf, &tmp_mask);
		if ((i + 1) < num_clusters &&
		    (total_cnt + cnt + 1) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			tmp[cnt-1] = ':';
			tmp[cnt] = '\0';
			total_cnt += cnt;
		} else if ((i + 1) == num_clusters &&
			   (total_cnt + cnt) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			total_cnt += cnt;
		} else {
			pr_err("invalid string for managed_cpu:%s%s\n", tmp,
				buf);
			break;
		}
	}
	snprintf(buf, PAGE_SIZE, "%s", tmp);
	return total_cnt;
}

static const struct kernel_param_ops param_ops_managed_online_cpus = {
	.get = get_managed_online_cpus,
};

/*******************************sysfs start************************************/
static int set_cpu_min_freq(const char *buf, const struct kernel_param *kp)
{
#if 0
	int i, j, ntokens = 0;
	unsigned int val, cpu;
	const char *cp = buf;
	struct cpu_status *i_cpu_stats;
	struct cpufreq_policy policy;
	cpumask_var_t limit_mask;
	int ret;
	const char *reset = "0:0 2:0";

 	if (touchboost == 0)
 		cp = reset;

	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	/* CPU:value pair */
	if (!(ntokens % 2))
		return -EINVAL;

	if (touchboost == 0)
 		cp = reset;
 	else
 		cp = buf;

	cpumask_clear(limit_mask);
	for (i = 0; i < ntokens; i += 2) {
		if (sscanf(cp, "%u:%u", &cpu, &val) != 2)
			return -EINVAL;
		if (cpu > (num_present_cpus() - 1))
			return -EINVAL;

		i_cpu_stats = &per_cpu(msm_perf_cpu_stats, cpu);

		i_cpu_stats->min = val;
		cpumask_set_cpu(cpu, limit_mask);

		cp = strnchr(cp, strlen(cp), ' ');
		cp++;
	}

	/*
	 * Since on synchronous systems policy is shared amongst multiple
	 * CPUs only one CPU needs to be updated for the limit to be
	 * reflected for the entire cluster. We can avoid updating the policy
	 * of other CPUs in the cluster once it is done for at least one CPU
	 * in the cluster
	 */
	get_online_cpus();
	for_each_cpu(i, limit_mask) {
		i_cpu_stats = &per_cpu(msm_perf_cpu_stats, i);

		if (cpufreq_get_policy(&policy, i))
			continue;

		if (cpu_online(i) && (policy.min != i_cpu_stats->min))
			cpufreq_update_policy(i);

		for_each_cpu(j, policy.related_cpus)
			cpumask_clear_cpu(j, limit_mask);
	}
	put_online_cpus();
#endif

	return 0;
}

static int get_cpu_min_freq(char *buf, const struct kernel_param *kp)
{
	int cnt = 0, cpu;

	for_each_present_cpu(cpu) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:%u ", cpu,
				per_cpu(msm_perf_cpu_stats, cpu).min);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	return cnt;
}

static const struct kernel_param_ops param_ops_cpu_min_freq = {
	.set = set_cpu_min_freq,
	.get = get_cpu_min_freq,
};
module_param_cb(cpu_min_freq, &param_ops_cpu_min_freq, NULL, 0644);

static int set_cpu_max_freq(const char *buf, const struct kernel_param *kp)
{
#if 0
	int i, j, ntokens = 0;
	unsigned int val, cpu;
	const char *cp = buf;
	struct cpu_status *i_cpu_stats;
	struct cpufreq_policy policy;
	cpumask_var_t limit_mask;

	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	/* CPU:value pair */
	if (!(ntokens % 2))
		return -EINVAL;

	cp = buf;
	cpumask_clear(limit_mask);
	for (i = 0; i < ntokens; i += 2) {
		if (sscanf(cp, "%u:%u", &cpu, &val) != 2)
			return -EINVAL;
		if (cpu > (num_present_cpus() - 1))
			return -EINVAL;

		i_cpu_stats = &per_cpu(msm_perf_cpu_stats, cpu);

		i_cpu_stats->max = val;
		cpumask_set_cpu(cpu, limit_mask);

		cp = strnchr(cp, strlen(cp), ' ');
		cp++;
	}

	get_online_cpus();
	for_each_cpu(i, limit_mask) {
		i_cpu_stats = &per_cpu(msm_perf_cpu_stats, i);
		if (cpufreq_get_policy(&policy, i))
			continue;

		if (cpu_online(i) && (policy.max != i_cpu_stats->max))
			cpufreq_update_policy(i);

		for_each_cpu(j, policy.related_cpus)
			cpumask_clear_cpu(j, limit_mask);
	}
	put_online_cpus();
#endif

	return 0;
}

static int get_cpu_max_freq(char *buf, const struct kernel_param *kp)
{
	int cnt = 0, cpu;

	for_each_present_cpu(cpu) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:%u ", cpu,
				per_cpu(msm_perf_cpu_stats, cpu).max);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	return cnt;
}

static const struct kernel_param_ops param_ops_cpu_max_freq = {
	.set = set_cpu_max_freq,
	.get = get_cpu_max_freq,
};
module_param_cb(cpu_max_freq, &param_ops_cpu_max_freq, NULL, 0644);

static struct kobject *events_kobj;

static ssize_t show_cpu_hotplug(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "\n");
}
static struct kobj_attribute cpu_hotplug_attr =
__ATTR(cpu_hotplug, 0444, show_cpu_hotplug, NULL);

static struct attribute *events_attrs[] = {
	&cpu_hotplug_attr.attr,
	NULL,
};

static struct attribute_group events_attr_group = {
	.attrs = events_attrs,
};

static ssize_t show_big_nr(struct kobject *kobj,
			   struct kobj_attribute *attr,
			   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", aggr_big_nr);
}

static struct kobj_attribute big_nr_attr =
__ATTR(aggr_big_nr, 0444, show_big_nr, NULL);

static ssize_t show_top_load(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", aggr_top_load);
}

static struct kobj_attribute top_load_attr =
__ATTR(aggr_top_load, 0444, show_top_load, NULL);


static ssize_t show_top_load_cluster(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u %u %u\n",
					top_load[MIN], top_load[MID],
					top_load[MAX]);
}

static struct kobj_attribute cluster_top_load_attr =
__ATTR(top_load_cluster, 0444, show_top_load_cluster, NULL);

static ssize_t show_curr_cap_cluster(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u %u %u\n",
					curr_cap[MIN], curr_cap[MID],
					curr_cap[MAX]);
}

static struct kobj_attribute cluster_curr_cap_attr =
__ATTR(curr_cap_cluster, 0444, show_curr_cap_cluster, NULL);

static struct attribute *notify_attrs[] = {
	&big_nr_attr.attr,
	&top_load_attr.attr,
	&cluster_top_load_attr.attr,
	&cluster_curr_cap_attr.attr,
	NULL,
};

static struct attribute_group notify_attr_group = {
	.attrs = notify_attrs,
};
static struct kobject *notify_kobj;

/*******************************sysfs ends************************************/

static int perf_adjust_notify(struct notifier_block *nb, unsigned long val,
							void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int cpu = policy->cpu;
	struct cpu_status *cpu_st = &per_cpu(msm_perf_cpu_stats, cpu);
	unsigned int min = cpu_st->min, max = cpu_st->max;


	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	pr_debug("msm_perf: CPU%u policy before: %u:%u kHz\n", cpu,
						policy->min, policy->max);
	pr_debug("msm_perf: CPU%u seting min:max %u:%u kHz\n", cpu, min, max);

	cpufreq_verify_within_limits(policy, min, max);

	pr_debug("msm_perf: CPU%u policy after: %u:%u kHz\n", cpu,
						policy->min, policy->max);

	return NOTIFY_OK;
}

static struct notifier_block perf_cpufreq_nb = {
	.notifier_call = perf_adjust_notify,
};

static int hotplug_notify(unsigned int cpu)
{
	unsigned long flags;

	if (events_group.init_success) {
		spin_lock_irqsave(&(events_group.cpu_hotplug_lock), flags);
		events_group.cpu_hotplug = true;
		spin_unlock_irqrestore(&(events_group.cpu_hotplug_lock), flags);
		wake_up_process(events_notify_thread);
	}

	return 0;
}

static int events_notify_userspace(void *data)
{
	unsigned long flags;
	bool notify_change;

	while (1) {

		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&(events_group.cpu_hotplug_lock), flags);

		if (!events_group.cpu_hotplug) {
			spin_unlock_irqrestore(&(events_group.cpu_hotplug_lock),
									flags);

			schedule();
			if (kthread_should_stop())
				break;
			spin_lock_irqsave(&(events_group.cpu_hotplug_lock),
									flags);
		}

		set_current_state(TASK_RUNNING);
		notify_change = events_group.cpu_hotplug;
		events_group.cpu_hotplug = false;
		spin_unlock_irqrestore(&(events_group.cpu_hotplug_lock), flags);

		if (notify_change)
			sysfs_notify(events_kobj, NULL, "cpu_hotplug");
	}

	return 0;
}

static int init_notify_group(void)
{
	int ret;
	struct kobject *module_kobj;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("msm_perf: Couldn't find module kobject\n");
		return -ENOENT;
	}

	notify_kobj = kobject_create_and_add("notify", module_kobj);
	if (!notify_kobj) {
		pr_err("msm_perf: Failed to add notify_kobj\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(notify_kobj, &notify_attr_group);
	if (ret) {
		kobject_put(notify_kobj);
		pr_err("msm_perf: Failed to create sysfs\n");
		return ret;
	}
	return 0;
}

static int init_events_group(void)
{
	int ret;
	struct kobject *module_kobj;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("msm_perf: Couldn't find module kobject\n");
		return -ENOENT;
	}

	events_kobj = kobject_create_and_add("events", module_kobj);
	if (!events_kobj) {
		pr_err("msm_perf: Failed to add events_kobj\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(events_kobj, &events_attr_group);
	if (ret) {
		pr_err("msm_perf: Failed to create sysfs\n");
		return ret;
	}

	spin_lock_init(&(events_group.cpu_hotplug_lock));
	events_notify_thread = kthread_run(events_notify_userspace,
					NULL, "msm_perf:events_notify");
	if (IS_ERR(events_notify_thread))
		return PTR_ERR(events_notify_thread);

	events_group.init_success = true;

	return 0;
}

static int __init msm_performance_init(void)
{
	unsigned int cpu;
	int rc;

	cpufreq_register_notifier(&perf_cpufreq_nb, CPUFREQ_POLICY_NOTIFIER);

	for_each_present_cpu(cpu)
		per_cpu(msm_perf_cpu_stats, cpu).max = UINT_MAX;

	rc = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE,
		"msm_performance_cpu_hotplug",
		hotplug_notify,
		NULL);

	init_events_group();
	init_notify_group();

	return 0;
}
late_initcall(msm_performance_init);
