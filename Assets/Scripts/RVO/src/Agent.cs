/*
 * Agent.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
 */

using System;
using System.Collections.Generic;

namespace RVO
{
    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    internal class Agent
    {
        internal IList<KeyValuePair<float, Agent>> agentNeighbors_ = new List<KeyValuePair<float, Agent>>();
        internal IList<KeyValuePair<float, Obstacle>> obstacleNeighbors_ = new List<KeyValuePair<float, Obstacle>>();
        internal IList<Line> orcaLines_ = new List<Line>();
        internal Vector2 position_;
        internal Vector2 prefVelocity_;
        internal Vector2 velocity_;
        internal int id_ = 0;
        internal int maxNeighbors_ = 0;
        internal float maxSpeed_ = 0.0f;
        internal float neighborDist_ = 0.0f;
        internal float radius_ = 0.0f;
        internal float timeHorizon_ = 0.0f;
        internal float timeHorizonObst_ = 0.0f;
        internal bool needDelete_ = false;

        private Vector2 newVelocity_;

        /**
         * <summary>Computes the neighbors of this agent.</summary>
         */
        internal void computeNeighbors()
        {
            obstacleNeighbors_.Clear();
            float rangeSq = RVOMath.sqr(timeHorizonObst_ * maxSpeed_ + radius_);
            Simulator.Instance.kdTree_.computeObstacleNeighbors(this, rangeSq);

            agentNeighbors_.Clear();

            if (maxNeighbors_ > 0)
            {
                rangeSq = RVOMath.sqr(neighborDist_);
                Simulator.Instance.kdTree_.computeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>Computes the new velocity of this agent.</summary>
         */
        internal void computeNewVelocity()
        {
            orcaLines_.Clear();

            // 第一阶段：为障碍物创建 ORCA 约束线
            CreateObstacleORCALines();

            int numObstLines = orcaLines_.Count;

            // 第二阶段：为其他 Agent 创建 ORCA 约束线
            CreateAgentORCALines();

            // 第三阶段：求解线性规划，找到最优速度
            SolveLinearProgram(numObstLines);
        }

        /// <summary>
        /// 为所有障碍物邻居创建 ORCA 约束线
        /// </summary>
        private void CreateObstacleORCALines()
        {
            float invTimeHorizonObst = 1.0f / timeHorizonObst_;

            for (int i = 0; i < obstacleNeighbors_.Count; ++i)
            {
                Obstacle obstacle1 = obstacleNeighbors_[i].Value;
                Obstacle obstacle2 = obstacle1.next_;

                Vector2 relativePosition1 = obstacle1.point_ - position_;
                Vector2 relativePosition2 = obstacle2.point_ - position_;

                // 检查该障碍物是否已被之前的 ORCA 线覆盖
                if (IsObstacleAlreadyCovered(relativePosition1, relativePosition2, invTimeHorizonObst))
                {
                    continue;
                }

                // 检查碰撞情况并添加相应的 ORCA 线
                if (TryAddCollisionORCALine(obstacle1, obstacle2, relativePosition1, relativePosition2))
                {
                    continue;
                }

                // 无碰撞情况：计算速度障碍物的腿（legs）
                ComputeAndAddVelocityObstacleORCALine(
                    obstacle1, obstacle2, 
                    relativePosition1, relativePosition2, 
                    invTimeHorizonObst
                );
            }
        }

        /// <summary>
        /// 检查障碍物是否已被现有的 ORCA 线覆盖
        /// </summary>
        private bool IsObstacleAlreadyCovered(Vector2 relativePosition1, Vector2 relativePosition2, float invTimeHorizonObst)
        {
            for (int j = 0; j < orcaLines_.Count; ++j)
            {
                float det1 = RVOMath.det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction);
                float det2 = RVOMath.det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction);
                
                if (det1 - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON && 
                    det2 - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// 尝试为碰撞情况添加 ORCA 线（顶点碰撞或边碰撞）
        /// </summary>
        /// <returns>如果检测到碰撞并添加了 ORCA 线则返回 true</returns>
        private bool TryAddCollisionORCALine(
            Obstacle obstacle1, Obstacle obstacle2,
            Vector2 relativePosition1, Vector2 relativePosition2)
        {
            float distSq1 = RVOMath.absSq(relativePosition1);
            float distSq2 = RVOMath.absSq(relativePosition2);
            float radiusSq = RVOMath.sqr(radius_);

            Vector2 obstacleVector = obstacle2.point_ - obstacle1.point_;
            float s = (-relativePosition1 * obstacleVector) / RVOMath.absSq(obstacleVector);
            float distSqLine = RVOMath.absSq(-relativePosition1 - s * obstacleVector);

            Line line;

            // 情况1：与左顶点碰撞
            if (s < 0.0f && distSq1 <= radiusSq)
            {
                if (obstacle1.convex_)
                {
                    line.point = new Vector2(0.0f, 0.0f);
                    line.direction = RVOMath.normalize(new Vector2(-relativePosition1.y(), relativePosition1.x()));
                    orcaLines_.Add(line);
                }
                return true;
            }

            // 情况2：与右顶点碰撞
            if (s > 1.0f && distSq2 <= radiusSq)
            {
                if (obstacle2.convex_ && RVOMath.det(relativePosition2, obstacle2.direction_) >= 0.0f)
                {
                    line.point = new Vector2(0.0f, 0.0f);
                    line.direction = RVOMath.normalize(new Vector2(-relativePosition2.y(), relativePosition2.x()));
                    orcaLines_.Add(line);
                }
                return true;
            }

            // 情况3：与障碍物边碰撞
            if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq)
            {
                line.point = new Vector2(0.0f, 0.0f);
                line.direction = -obstacle1.direction_;
                orcaLines_.Add(line);
                return true;
            }

            return false;
        }

        /// <summary>
        /// 计算并添加速度障碍物的 ORCA 线（无碰撞情况）
        /// </summary>
        private void ComputeAndAddVelocityObstacleORCALine(
            Obstacle obstacle1, Obstacle obstacle2,
            Vector2 relativePosition1, Vector2 relativePosition2,
            float invTimeHorizonObst)
        {
            float distSq1 = RVOMath.absSq(relativePosition1);
            float distSq2 = RVOMath.absSq(relativePosition2);
            float radiusSq = RVOMath.sqr(radius_);

            Vector2 obstacleVector = obstacle2.point_ - obstacle1.point_;
            float s = (-relativePosition1 * obstacleVector) / RVOMath.absSq(obstacleVector);
            float distSqLine = RVOMath.absSq(-relativePosition1 - s * obstacleVector);

            // 计算速度障碍物的左右腿方向
            Vector2 leftLegDirection, rightLegDirection;

            if (s < 0.0f && distSqLine <= radiusSq)
            {
                // 斜视障碍物，左顶点定义速度障碍物
                if (!obstacle1.convex_) return;
                
                obstacle2 = obstacle1;
                float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                leftLegDirection = ComputeLegDirection(relativePosition1, leg1, radius_, distSq1, true);
                rightLegDirection = ComputeLegDirection(relativePosition1, leg1, radius_, distSq1, false);
            }
            else if (s > 1.0f && distSqLine <= radiusSq)
            {
                // 斜视障碍物，右顶点定义速度障碍物
                if (!obstacle2.convex_) return;
                
                obstacle1 = obstacle2;
                float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                leftLegDirection = ComputeLegDirection(relativePosition2, leg2, radius_, distSq2, true);
                rightLegDirection = ComputeLegDirection(relativePosition2, leg2, radius_, distSq2, false);
            }
            else
            {
                // 常规情况：障碍物正面或侧面视角
                ComputeNormalLegs(
                    obstacle1, obstacle2,
                    relativePosition1, relativePosition2,
                    distSq1, distSq2, radiusSq,
                    out leftLegDirection, out rightLegDirection
                );
            }

            // 调整腿方向，避免指向相邻障碍物
            AdjustLegsForNeighboringObstacles(
                obstacle1, obstacle2,
                ref leftLegDirection, ref rightLegDirection,
                out bool isLeftLegForeign, out bool isRightLegForeign
            );

            // 投影当前速度并选择最佳 ORCA 线
            ProjectVelocityAndAddORCALine(
                obstacle1, obstacle2,
                leftLegDirection, rightLegDirection,
                isLeftLegForeign, isRightLegForeign,
                invTimeHorizonObst
            );
        }

        /// <summary>
        /// 计算速度障碍物的腿方向（左腿或右腿）
        /// </summary>
        private Vector2 ComputeLegDirection(Vector2 relativePosition, float legLength, float radius, float distSq, bool isLeftLeg)
        {
            if (isLeftLeg)
            {
                return new Vector2(
                    relativePosition.x() * legLength - relativePosition.y() * radius,
                    relativePosition.x() * radius + relativePosition.y() * legLength
                ) / distSq;
            }
            else
            {
                return new Vector2(
                    relativePosition.x() * legLength + relativePosition.y() * radius,
                    -relativePosition.x() * radius + relativePosition.y() * legLength
                ) / distSq;
            }
        }

        /// <summary>
        /// 计算常规情况下的左右腿方向
        /// </summary>
        private void ComputeNormalLegs(
            Obstacle obstacle1, Obstacle obstacle2,
            Vector2 relativePosition1, Vector2 relativePosition2,
            float distSq1, float distSq2, float radiusSq,
            out Vector2 leftLegDirection, out Vector2 rightLegDirection)
        {
            // 计算左腿
            if (obstacle1.convex_)
            {
                float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                leftLegDirection = ComputeLegDirection(relativePosition1, leg1, radius_, distSq1, true);
            }
            else
            {
                // 左顶点非凸，左腿延伸切断线
                leftLegDirection = -obstacle1.direction_;
            }

            // 计算右腿
            if (obstacle2.convex_)
            {
                float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                rightLegDirection = ComputeLegDirection(relativePosition2, leg2, radius_, distSq2, false);
            }
            else
            {
                // 右顶点非凸，右腿延伸切断线
                rightLegDirection = obstacle1.direction_;
            }
        }

        /// <summary>
        /// 调整腿方向以避免指向相邻障碍物
        /// </summary>
        private void AdjustLegsForNeighboringObstacles(
            Obstacle obstacle1, Obstacle obstacle2,
            ref Vector2 leftLegDirection, ref Vector2 rightLegDirection,
            out bool isLeftLegForeign, out bool isRightLegForeign)
        {
            Obstacle leftNeighbor = obstacle1.previous_;

            isLeftLegForeign = false;
            isRightLegForeign = false;

            // 检查左腿是否指向障碍物内部
            if (obstacle1.convex_ && RVOMath.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0f)
            {
                leftLegDirection = -leftNeighbor.direction_;
                isLeftLegForeign = true;
            }

            // 检查右腿是否指向障碍物内部
            if (obstacle2.convex_ && RVOMath.det(rightLegDirection, obstacle2.direction_) <= 0.0f)
            {
                rightLegDirection = obstacle2.direction_;
                isRightLegForeign = true;
            }
        }

        /// <summary>
        /// 将当前速度投影到速度障碍物上并添加最合适的 ORCA 线
        /// </summary>
        private void ProjectVelocityAndAddORCALine(
            Obstacle obstacle1, Obstacle obstacle2,
            Vector2 leftLegDirection, Vector2 rightLegDirection,
            bool isLeftLegForeign, bool isRightLegForeign,
            float invTimeHorizonObst)
        {
            // 计算切断中心点
            Vector2 leftCutOff = invTimeHorizonObst * (obstacle1.point_ - position_);
            Vector2 rightCutOff = invTimeHorizonObst * (obstacle2.point_ - position_);
            Vector2 cutOffVector = rightCutOff - leftCutOff;

            // 投影当前速度到速度障碍物
            float t = obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutOff) * cutOffVector) / RVOMath.absSq(cutOffVector);
            float tLeft = (velocity_ - leftCutOff) * leftLegDirection;
            float tRight = (velocity_ - rightCutOff) * rightLegDirection;

            Line line;

            // 情况1：投影到左切断圆
            if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f))
            {
                Vector2 unitW = RVOMath.normalize(velocity_ - leftCutOff);
                line.direction = new Vector2(unitW.y(), -unitW.x());
                line.point = leftCutOff + radius_ * invTimeHorizonObst * unitW;
                orcaLines_.Add(line);
                return;
            }

            // 情况2：投影到右切断圆
            if (t > 1.0f && tRight < 0.0f)
            {
                Vector2 unitW = RVOMath.normalize(velocity_ - rightCutOff);
                line.direction = new Vector2(unitW.y(), -unitW.x());
                line.point = rightCutOff + radius_ * invTimeHorizonObst * unitW;
                orcaLines_.Add(line);
                return;
            }

            // 情况3：选择最近的投影（切断线、左腿或右腿）
            float distSqCutoff = (t < 0.0f || t > 1.0f || obstacle1 == obstacle2) 
                ? float.PositiveInfinity 
                : RVOMath.absSq(velocity_ - (leftCutOff + t * cutOffVector));
            
            float distSqLeft = tLeft < 0.0f 
                ? float.PositiveInfinity 
                : RVOMath.absSq(velocity_ - (leftCutOff + tLeft * leftLegDirection));
            
            float distSqRight = tRight < 0.0f 
                ? float.PositiveInfinity 
                : RVOMath.absSq(velocity_ - (rightCutOff + tRight * rightLegDirection));

            // 投影到切断线
            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
            {
                line.direction = -obstacle1.direction_;
                line.point = leftCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y(), line.direction.x());
                orcaLines_.Add(line);
                return;
            }

            // 投影到左腿
            if (distSqLeft <= distSqRight)
            {
                if (isLeftLegForeign) return;

                line.direction = leftLegDirection;
                line.point = leftCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y(), line.direction.x());
                orcaLines_.Add(line);
                return;
            }

            // 投影到右腿
            if (isRightLegForeign) return;

            line.direction = -rightLegDirection;
            line.point = rightCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y(), line.direction.x());
            orcaLines_.Add(line);
        }

        /// <summary>
        /// 为所有 Agent 邻居创建 ORCA 约束线
        /// </summary>
        private void CreateAgentORCALines()
        {
            float invTimeHorizon = 1.0f / timeHorizon_;

            for (int i = 0; i < agentNeighbors_.Count; ++i)
            {
                Agent other = agentNeighbors_[i].Value;

                Vector2 relativePosition = other.position_ - position_;
                Vector2 relativeVelocity = velocity_ - other.velocity_;
                float distSq = RVOMath.absSq(relativePosition);
                float combinedRadius = radius_ + other.radius_;
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Line line;
                Vector2 u;

                if (distSq > combinedRadiusSq)
                {
                    // 无碰撞：应用 ORCA 互惠避让
                    u = ComputeORCAVectorNoCollision(
                        relativePosition, relativeVelocity, 
                        distSq, combinedRadius, combinedRadiusSq,
                        invTimeHorizon, out line
                    );
                }
                else
                {
                    // 碰撞：紧急避让
                    u = ComputeORCAVectorCollision(
                        relativePosition, relativeVelocity,
                        combinedRadius, out line
                    );
                }

                // ORCA 线的点位于当前速度加上一半的避让向量（互惠原则）
                line.point = velocity_ + 0.5f * u;
                orcaLines_.Add(line);
            }
        }

        /// <summary>
        /// 计算无碰撞情况下的 ORCA 避让向量
        /// </summary>
        private Vector2 ComputeORCAVectorNoCollision(
            Vector2 relativePosition, Vector2 relativeVelocity,
            float distSq, float combinedRadius, float combinedRadiusSq,
            float invTimeHorizon, out Line line)
        {
            // w 是从切断中心到相对速度的向量
            Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
            float wLengthSq = RVOMath.absSq(w);
            float dotProduct1 = w * relativePosition;

            // 判断是投影到切断圆还是投影到腿
            if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
            {
                // 投影到切断圆
                float wLength = RVOMath.sqrt(wLengthSq);
                Vector2 unitW = w / wLength;

                line.direction = new Vector2(unitW.y(), -unitW.x());
                line.point = new Vector2(0.0f, 0.0f);
                return (combinedRadius * invTimeHorizon - wLength) * unitW;
            }
            else
            {
                // 投影到腿（左腿或右腿）
                float leg = RVOMath.sqrt(distSq - combinedRadiusSq);

                if (RVOMath.det(relativePosition, w) > 0.0f)
                {
                    // 投影到左腿
                    line.direction = new Vector2(
                        relativePosition.x() * leg - relativePosition.y() * combinedRadius,
                        relativePosition.x() * combinedRadius + relativePosition.y() * leg
                    ) / distSq;
                }
                else
                {
                    // 投影到右腿
                    line.direction = -new Vector2(
                        relativePosition.x() * leg + relativePosition.y() * combinedRadius,
                        -relativePosition.x() * combinedRadius + relativePosition.y() * leg
                    ) / distSq;
                }

                float dotProduct2 = relativeVelocity * line.direction;
                line.point = new Vector2(0.0f, 0.0f);
                return dotProduct2 * line.direction - relativeVelocity;
            }
        }

        /// <summary>
        /// 计算碰撞情况下的 ORCA 避让向量（紧急避让）
        /// </summary>
        private Vector2 ComputeORCAVectorCollision(
            Vector2 relativePosition, Vector2 relativeVelocity,
            float combinedRadius, out Line line)
        {
            float invTimeStep = 1.0f / Simulator.Instance.timeStep_;

            // 使用时间步长的切断圆（比时间视野更紧急）
            Vector2 w = relativeVelocity - invTimeStep * relativePosition;
            float wLength = RVOMath.abs(w);
            Vector2 unitW = w / wLength;

            line.direction = new Vector2(unitW.y(), -unitW.x());
            line.point = new Vector2(0.0f, 0.0f);
            return (combinedRadius * invTimeStep - wLength) * unitW;
        }

        /// <summary>
        /// 求解线性规划问题，找到最优的新速度
        /// </summary>
        private void SolveLinearProgram(int numObstLines)
        {
            // 第一步：尝试用线性规划2求解（二维优化）
            int lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, ref newVelocity_);

            // 第二步：如果失败，使用线性规划3求解（三维优化，允许违反部分约束）
            if (lineFail < orcaLines_.Count)
            {
                linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, ref newVelocity_);
            }
        }

        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertAgentNeighbor(Agent agent, ref float rangeSq)
        {
            if (this != agent)
            {
                float distSq = RVOMath.absSq(position_ - agent.position_);

                if (distSq < rangeSq)
                {
                    if (agentNeighbors_.Count < maxNeighbors_)
                    {
                        agentNeighbors_.Add(new KeyValuePair<float, Agent>(distSq, agent));
                    }

                    int i = agentNeighbors_.Count - 1;

                    while (i != 0 && distSq < agentNeighbors_[i - 1].Key)
                    {
                        agentNeighbors_[i] = agentNeighbors_[i - 1];
                        --i;
                    }

                    agentNeighbors_[i] = new KeyValuePair<float, Agent>(distSq, agent);

                    if (agentNeighbors_.Count == maxNeighbors_)
                    {
                        rangeSq = agentNeighbors_[agentNeighbors_.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertObstacleNeighbor(Obstacle obstacle, float rangeSq)
        {
            Obstacle nextObstacle = obstacle.next_;

            float distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position_);

            if (distSq < rangeSq)
            {
                obstacleNeighbors_.Add(new KeyValuePair<float, Obstacle>(distSq, obstacle));

                int i = obstacleNeighbors_.Count - 1;

                while (i != 0 && distSq < obstacleNeighbors_[i - 1].Key)
                {
                    obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                    --i;
                }
                obstacleNeighbors_[i] = new KeyValuePair<float, Obstacle>(distSq, obstacle);
            }
        }

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void update()
        {
            velocity_ = newVelocity_;
            position_ += velocity_ * Simulator.Instance.timeStep_;
        }

        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private bool linearProgram1(IList<Line> lines, int lineNo, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            float dotProduct = lines[lineNo].point * lines[lineNo].direction;
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = RVOMath.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                float numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (RVOMath.fabs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (optVelocity * lines[lineNo].direction > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private int linearProgram2(IList<Line> lines, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, float radius, ref Vector2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (RVOMath.fabs(determinant) <= RVOMath.RVO_EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (lines[i].direction * lines[j].direction > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5f * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    Vector2 tempResult = result;
                    if (linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, ref result) < projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
    }
}
