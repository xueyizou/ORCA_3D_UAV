/*
 *This code was derived from the RVO2 Library C++ , which is Copyright (c) 2008-2011 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 * 
 * This Java implementation was done by Xueyi Zou at the University of York, UK.
 * Xueyi Zou can be contacted via xz972@york.ac.uk
 */

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
