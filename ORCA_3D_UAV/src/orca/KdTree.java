package orca;

import java.util.ArrayList;

import sim.util.MutableDouble3D;

public class KdTree {
	
	final int RVO_MAX_LEAF_SIZE = 10;
	
	RVO2 ptrsim_;
	ArrayList<Agent > agents_;
	ArrayList<AgentTreeNode> agentTree_=new ArrayList<AgentTreeNode>();
	
	/**
	 * \brief   finalructs a <i>k</i>d-tree instance.
	 * \param   sim  The simulator instance.
	 */
	public KdTree(RVO2 rvoSimulator) {
		this.ptrsim_=rvoSimulator;
	}

	/**
	 * \brief   Builds an agent <i>k</i>d-tree.
	 */
	void buildAgentTree(){
		agents_ = ptrsim_.agents_;

		if (!agents_.isEmpty()) {
			for(int i=0; i<2 * agents_.size() - 1; ++i)//	agentTree_.setSize(2 * agents_.size() - 1);
			{
				agentTree_.add(new AgentTreeNode());
			}
			buildAgentTreeRecursive(0, agents_.size(), 0);
		}
	}

	void buildAgentTreeRecursive(int begin, int end, int node){
		agentTree_.get(node).begin = begin;
		agentTree_.get(node).end = end;
		agentTree_.get(node).minCoord = new MutableDouble3D(agents_.get(begin).position_);
		agentTree_.get(node).maxCoord = new MutableDouble3D(agents_.get(begin).position_);

		for (int i = begin + 1; i < end; ++i) {
			agentTree_.get(node).maxCoord.x = Math.max(agentTree_.get(node).maxCoord.x, agents_.get(i).position_.x);
			agentTree_.get(node).minCoord.x = Math.min(agentTree_.get(node).minCoord.x, agents_.get(i).position_.x);
			agentTree_.get(node).maxCoord.y = Math.max(agentTree_.get(node).maxCoord.y, agents_.get(i).position_.y);
			agentTree_.get(node).minCoord.y = Math.min(agentTree_.get(node).minCoord.y, agents_.get(i).position_.y);
			agentTree_.get(node).maxCoord.z = Math.max(agentTree_.get(node).maxCoord.z, agents_.get(i).position_.z);
			agentTree_.get(node).minCoord.z = Math.min(agentTree_.get(node).minCoord.z, agents_.get(i).position_.z);
		}

		if (end - begin > RVO_MAX_LEAF_SIZE) {
			/* No leaf node. */
			int coord;

			if (agentTree_.get(node).maxCoord.x - agentTree_.get(node).minCoord.x > agentTree_.get(node).maxCoord.y - agentTree_.get(node).minCoord.y && agentTree_.get(node).maxCoord.x - agentTree_.get(node).minCoord.x > agentTree_.get(node).maxCoord.z - agentTree_.get(node).minCoord.z) {
				coord = 0;
			}
			else if (agentTree_.get(node).maxCoord.y - agentTree_.get(node).minCoord.y > agentTree_.get(node).maxCoord.z - agentTree_.get(node).minCoord.z) {
				coord = 1;
			}
			else {
				coord = 2;
			}

			final double splitValue = (double) (0.5f * (agentTree_.get(node).maxCoord.getElement(coord) + agentTree_.get(node).minCoord.getElement(coord)));

			int left = begin;

			int right = end;

			while (left < right) {
				while (left < right && agents_.get(left).position_.getElement(coord) < splitValue) {
					++left;
				}

				while (right > left && agents_.get(right - 1).position_.getElement(coord) >= splitValue) {
					--right;
				}

				if (left < right) {
					Utils.swap(agents_.get(left), agents_.get(right - 1));
					++left;
					--right;
				}
			}

			int leftSize = left - begin;

			if (leftSize == 0) {
				++leftSize;
				++left;
				++right;
			}

			agentTree_.get(node).left = node + 1;
			agentTree_.get(node).right = node + 2 * leftSize;

			buildAgentTreeRecursive(begin, left, agentTree_.get(node).left);
			buildAgentTreeRecursive(left, end, agentTree_.get(node).right);
		}
	}


	/**
	 * \brief   Computes the agent neighbors of the specified agent.
	 * \param   agent    A pointer to the agent for which agent neighbors are to be computed.
	 * \param   rangeSq  The squared range around the agent.
	 */
	void computeAgentNeighbors(Agent agent, double rangeSq){
		MutableDouble mutableRangeSq = new MutableDouble(rangeSq);
		queryAgentTreeRecursive(agent, mutableRangeSq, 0);
	}


	void queryAgentTreeRecursive(Agent agent, MutableDouble rangeSq, int node){
		if (agentTree_.get(node).end - agentTree_.get(node).begin <= RVO_MAX_LEAF_SIZE) {
			for (int i = agentTree_.get(node).begin; i < agentTree_.get(node).end; ++i) {
				agent.insertAgentNeighbor(agents_.get(i), rangeSq);
			}
		}
		else {
			final double distSqLeft = (double) (Utils.Sq(Math.max(0.0f, agentTree_.get(agentTree_.get(node).left).minCoord.x - agent.position_.x)) + Utils.Sq(Math.max(0.0f, agent.position_.x - agentTree_.get(agentTree_.get(node).left).maxCoord.x)) + Utils.Sq(Math.max(0.0f, agentTree_.get(agentTree_.get(node).left).minCoord.y - agent.position_.y)) + Utils.Sq(Math.max(0.0f, agent.position_.y - agentTree_.get(agentTree_.get(node).left).maxCoord.y)) + Utils.Sq(Math.max(0.0f, agentTree_.get(agentTree_.get(node).left).minCoord.z - agent.position_.z)) + Utils.Sq(Math.max(0.0f, agent.position_.z - agentTree_.get(agentTree_.get(node).left).maxCoord.z)));

			final double distSqRight = (double) (Utils.Sq(Math.max(0.0f, agentTree_.get(agentTree_.get(node).right).minCoord.x - agent.position_.x)) + Utils.Sq(Math.max(0.0f, agent.position_.x - agentTree_.get(agentTree_.get(node).right).maxCoord.x)) + Utils.Sq(Math.max(0.0f, agentTree_.get(agentTree_.get(node).right).minCoord.y - agent.position_.y)) + Utils.Sq(Math.max(0.0f, agent.position_.y - agentTree_.get(agentTree_.get(node).right).maxCoord.y)) + Utils.Sq(Math.max(0.0f, agentTree_.get(agentTree_.get(node).right).minCoord.z - agent.position_.z)) + Utils.Sq(Math.max(0.0f, agent.position_.z - agentTree_.get(agentTree_.get(node).right).maxCoord.z)));

			if (distSqLeft < distSqRight) {
				if (distSqLeft < rangeSq.doubleValue) {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_.get(node).left);

					if (distSqRight < rangeSq.doubleValue) {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_.get(node).right);
					}
				}
			}
			else {
				if (distSqRight < rangeSq.doubleValue) {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_.get(node).right);
					
					if (distSqLeft < rangeSq.doubleValue) {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_.get(node).left);
					}
				}
			}
		}
	}
}

/**
 * \brief   Defines an agent <i>k</i>d-tree node.
 */
class AgentTreeNode {
	/**
	 * \brief   The beginning node number.
	 */
	public int begin;

	/**
	 * \brief   The ending node number.
	 */
	public int end;

	/**
	 * \brief   The left node number.
	 */
	public int left;

	/**
	 * \brief   The right node number.
	 */
	public int right;

	/**
	 * \brief   The maximum coordinates.
	 */
	public MutableDouble3D maxCoord;

	/**
	 * \brief   The minimum coordinates.
	 */
	public MutableDouble3D minCoord;
}


class MutableDouble
{
	public double doubleValue;
	
	public MutableDouble()
	{
		this.doubleValue = 0.0;
	}
	
	public MutableDouble(double value)
	{
		this.doubleValue = value;
	}
}
