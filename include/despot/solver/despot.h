#ifndef DESPOT_H
#define DESPOT_H

#include <despot/core/solver.h>
#include <despot/core/pomdp.h>
#include <despot/core/belief.h>
#include <despot/core/node.h>
#include <despot/core/globals.h>
#include <despot/core/history.h>
#include <despot/random_streams.h>

namespace despot {

class DESPOT: public Solver {
friend class VNode;

protected:
  Config& config_;
	VNode* root_;
	SearchStatistics statistics_;

	ScenarioLowerBound* lower_bound_;
	ScenarioUpperBound* upper_bound_;

public:
	DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL);
	virtual ~DESPOT();

	ValuedAction Search();

	void belief(Belief* b);
	void Update(int action, OBS_TYPE obs);

	ScenarioLowerBound* lower_bound() const;
	ScenarioUpperBound* upper_bound() const;

	static VNode* ConstructTree(std::vector<State*>& particles, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, double timeout,
    const Config& config, SearchStatistics* statistics = NULL);

protected:
	static VNode* Trial(VNode* root, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, const Config& config,
    SearchStatistics* statistics = NULL);
	static void InitLowerBound(VNode* vnode, ScenarioLowerBound* lower_bound,
		RandomStreams& streams, History& history, const Config& config);
	static void InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
		RandomStreams& streams, History& history, const Config& config);
	static void InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, RandomStreams& streams, History& history,
    const Config& config);

	static void Expand(VNode* vnode,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, RandomStreams& streams, History& history,
    const Config& config);
	static void Backup(VNode* vnode, const Config& config);

	static double Gap(VNode* vnode);

	double CheckDESPOT(const VNode* vnode, double regularized_value, const Config& config);
	double CheckDESPOTSTAR(const VNode* vnode, double regularized_value, const Config& config);
	void Compare();

	static void ExploitBlockers(VNode* vnode, const Config& config);
	static VNode* FindBlocker(VNode* vnode, const Config& config);
	static void Expand(QNode* qnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, const DSPOMDP* model,
		RandomStreams& streams, History& history, const Config& config);
	static void Update(VNode* vnode);
	static void Update(QNode* qnode, const Config& config);
	static VNode* Prune(VNode* vnode, int& pruned_action, double& pruned_value, const Config& config);
	static QNode* Prune(QNode* qnode, double& pruned_value, const Config& config);
	static double WEU(VNode* vnode, const Config& config);
	static double WEU(VNode* vnode, double epsilon);
	static VNode* SelectBestWEUNode(QNode* qnode, const Config& config);
	static QNode* SelectBestUpperBoundNode(VNode* vnode);
	static ValuedAction OptimalAction(VNode* vnode);

	static ValuedAction Evaluate(VNode* root, std::vector<State*>& particles,
		RandomStreams& streams, POMCPPrior* prior, const DSPOMDP* model, const Config& config);
};

} // namespace despot

#endif
