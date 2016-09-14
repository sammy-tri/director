import os
import math
import time
import types
import functools
import numpy as np

import bot_core as lcmbotcore
import drake as lcmdrake
import pydrake
import pydrake.solvers.ik as drakeik

import director

class IKPyDrake():
    """Provides an ik interface using pydrake.  Because the interface was
    originally implemented for a remote MATLAB interface, some legacy
    parts of the API remain.
    """

    def __init__(self, robotURDF, jointController, fixedPointFile, leftFootLink, rightFootLink, pelvisLink):

        self.robotURDF = robotURDF
        self.jointController = jointController
        self.fixedPointFile = fixedPointFile
        self.leftFootLink = leftFootLink
        self.rightFootLink = rightFootLink
        self.pelvisLink = pelvisLink

        self.seedName = 'q_nom'
        self.nominalName = 'q_nom'
        self.infoFunc = None

        # Setup pydrake IK.
        self.model = pydrake.rbtree.RigidBodyTree(str(robotURDF))

    def getFrozenGroupString(self):
        # TODO(sam.creasey) Enable this functionality in pydrake mode.
        frozenGroups = []
        if getattr(self,"leftArmLocked",False):
            frozenGroups.append("l_arm")
        if getattr(self,"rightArmLocked",False):
            frozenGroups.append("r_arm")
        if getattr(self,"baseLocked",False):
            frozenGroups.append("pelvis")
        if getattr(self,"backLocked",False):
            frozenGroups.append("back")
        if frozenGroups:
            return "{'" + "','".join(frozenGroups) + "'}"
        else:
            return "{}"

    def updateJointLimits(self, limitData):
        # TODO(sam.creasey) Do we need to implement this?  Or is the
        # posture constraint adequate to capture our joint limits?
        pass

    def setEnvironment(self, urdf_string):
        # TODO(sam.creasey) Implement this once a pydrake-using
        # configuration can handle the collision environment.
        print "Collision environment not supported with pydrake."

    def clearEnvironment(self):
        self.setEnvironment('')

    def runIk(self, constraints, ikParameters, nominalPostureName=None, seedPostureName=None):
        # TODO(sam.creasey) Should we warn if collision parameters
        # were specified which we can't honor? (such as MinDistance)
        options = drakeik.IKoptions(self.model)
        options.setMajorIterationsLimit(ikParameters.majorIterationsLimit)
        options.setMajorFeasibilityTolerance(ikParameters.majorFeasibilityTolerance)
        options.setMajorOptimalityTolerance(ikParameters.majorOptimalityTolerance)

        nominalPostureName = nominalPostureName or self.nominalName
        seedPostureName = seedPostureName or self.seedName
        print "nominal", nominalPostureName, self.jointController.getPose(nominalPostureName)
        print "seed", seedPostureName, self.jointController.getPose(seedPostureName)

        q_seed = self.jointController.getPose(seedPostureName)
        q_nom = self.jointController.getPose(nominalPostureName)
        drake_constraints = list()
        for c in constraints:
            drake_constraints.append(c.getDrakeConstraint(self.model))
        results = drakeik.InverseKin(
            self.model, q_seed, q_nom, drake_constraints, options)
        print results.q_sol[0], results.info[0]
        return results.q_sol[0], results.info[0]

    def runIkTraj(self, constraints, poseStart, poseEnd, nominalPose, ikParameters, timeSamples=None, additionalTimeSamples=0, graspToHandLinkFrame=None):

        if timeSamples is None:
            timeSamples = np.hstack([constraint.tspan for constraint in constraints])
            timeSamples = [x for x in timeSamples if x not in [-np.inf, np.inf]]
            timeSamples.append(0.0)
            timeSamples = np.unique(timeSamples).tolist()
            timeSamples += np.linspace(timeSamples[0], timeSamples[-1], ikParameters.numberOfAddedKnots + 2).tolist()
            timeSamples = np.unique(timeSamples).tolist()

        assert ikParameters.rrtHand in ('left', 'right')
        collisionEndEffectorName = ( self.handModels[0].handLinkName if ikParameters.rrtHand == 'left' else self.handModels[1].handLinkName )


        # TODO(sam.creasey) Should we warn if collision parameters
        # were specified which we can't honor? (such as MinDistance)
        options = drakeik.IKoptions(self.model)
        options.setMajorIterationsLimit(ikParameters.majorIterationsLimit)
        options.setMajorFeasibilityTolerance(ikParameters.majorFeasibilityTolerance)
        options.setMajorOptimalityTolerance(ikParameters.majorOptimalityTolerance)
        options.setFixInitialState(ikParameters.fixInitialState)

        print "start", poseStart, self.jointController.getPose(poseStart)
        print "end", poseEnd, self.jointController.getPose(poseEnd)
        print "nominal", nominalPose, self.jointController.getPose(nominalPose)

        seed_pose = self.jointController.getPose(poseStart)
        q_seed = np.array([[float(seed_pose[i]) for _ in xrange(len(timeSamples))] for i in xrange(len(seed_pose))])
        q_seed = np.transpose(np.array([seed_pose] * len(timeSamples)))
        nom_pose = self.jointController.getPose(nominalPose)
        q_nom = np.transpose(np.array([seed_pose] * len(timeSamples)))
        print "seed", q_seed

        # TODO(sam.creasey) Handle additional time samples.  The
        # previous MATLAB code did this:
        #
        # commands.append('additionalTimeSamples = linspace(t(1),
        # t(end), %d);' % additionalTimeSamples)
        ts_array = np.array([float(x) for x in timeSamples])
        print "time samples", timeSamples, ts_array
        print "additional", additionalTimeSamples

        drake_constraints = list()
        for c in constraints:
            drake_constraints.append(c.getDrakeConstraint(self.model))

        if ikParameters.usePointwise:
            results = drakeik.InverseKinPointwise(
                self.model, ts_array, q_seed, q_nom, drake_constraints, options)
        else:
            results = drakeik.InverseKinTraj(
                self.model, ts_array, q_seed, q_nom, drake_constraints, options)
        print "result states", len(results.q_sol), results.info[0]
        if self.infoFunc:
            self.infoFunc(results.info[0])
        plan = lcmdrake.robot_plan_t()

        plan.utime = time.time() * 1e6;  # TODO(sam.creasey) fix this
        plan.num_states = len(results.q_sol)
        plan.plan = []
        plan.plan_info = []
        joint_names = [self.model.getPositionName(x) for x in xrange(len(seed_pose))]
        print "joints", joint_names
        for idx in xrange(plan.num_states):
            q_sol = results.q_sol[idx]
            print "sol", idx, q_sol
            state = lcmbotcore.robot_state_t()
            state.utime = timeSamples[idx] * 1e6
            print "state ", idx, "utime", state.utime
            state.pose = lcmbotcore.position_3d_t()
            state.pose.translation = lcmbotcore.vector_3d_t()
            # TODO(sam.creasey) We might want real values for these
            # someday.  We can probably use the base_* model positions?
            state.pose.translation.x = 0
            state.pose.translation.y = 0
            state.pose.translation.z = 0.
            state.pose.rotation = lcmbotcore.quaternion_t()
            state.pose.rotation.w = 0.
            state.pose.rotation.x = 0.
            state.pose.rotation.y = 0.
            state.pose.rotation.z = 0.
            state.twist = lcmbotcore.twist_t()
            state.num_joints = len(q_sol)
            state.joint_name = joint_names
            state.joint_position = [float(q_sol[x]) for x in xrange(len(q_sol))]
            state.joint_velocity = [0.] * len(q_sol)
            state.joint_effort = [0.] * len(q_sol)
            plan.plan.append(state)
            if ikParameters.usePointwise:
                plan.plan_info.append(results.info[0])
            else:
                plan.plan_info.append(results.info[idx])

        for idx in xrange(plan.num_states):
            print "utime at ", idx, plan.plan[idx].utime

        return plan, results.info[0]

    def addAffordanceToLink(self, linkName, affordance, q, affordanceName):
        # TODO(sam.creasey) I'm not sure this even has any callers.
        print "Affordances not supported with pydrake."

    def removeAffordanceFromLink(self, linkName, affordanceName, nominalPoseName):
        # TODO(sam.creasey) I'm not sure this even has any callers.
        print "Affordances not supported with pydrake."

    def searchFinalPose(self, constraints, side, eeName, eePose, nominalPoseName, capabilityMapFile, ikParameters):
        # TODO(sam.creasey) This depends on the collision free
        # planner, which is only implemented in MATLAB.
        print "Final pose planning not supported with pydrake."
