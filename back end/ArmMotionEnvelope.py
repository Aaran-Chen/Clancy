"""Directional enclosure of an arm box over a revolute-joint interval.

The linear term uses each corner's actual joint Jacobian. A global Hessian
bound encloses the nonlinear remainder, so this is a certificate over the
whole interval, not a sampling heuristic. See Cad/BasketAvoidance.md.
"""
import itertools as Itertools
import numpy as Np
import CadKinematics as Cad

Signs = Np.array(list(Itertools.product((-1.0, 1.0), repeat=3)))
LaterJoint = Np.maximum.outer(Np.arange(len(Cad.JointNames)), Np.arange(len(Cad.JointNames)))


def BoxEnvelope(Center, Half, Radii, Transform, Frames, Motion):
    """World center, orientation and half-size enclosing all independently moving joints.

    Motion is the nonnegative angular half-width in radians, including tracking.
    Radii bounds the distance of every box point to each ancestor joint at any
    pose. Non-ancestor entries are zero. Joint order follows the serial chain.
    """
    Rotation, Origin = Transform[:3, :3], Transform[:3, 3]
    Corners = Center + Signs * Half
    Points = Corners @ Rotation.T + Origin
    Axes = Np.array([Frames[Name][:3, 2] for Name in Cad.JointNames])
    Origins = Np.array([Frames[Name][:3, 3] for Name in Cad.JointNames])
    Jacobian = Np.cross(Axes[None, :, :], Points[:, None, :] - Origins[None, :, :]) @ Rotation
    Jacobian[:, Radii == 0, :] = 0
    Error = Np.einsum("cji,j->ci", Np.abs(Jacobian), Motion)
    # Differentiating the rigid-transform product twice leaves a vector from
    # the later joint to the point. Rotation derivatives have operator norm 1,
    # so its norm is <= that joint's radius, including mixed derivatives.
    Hessian = Radii[LaterJoint].copy()
    Hessian[Radii == 0, :] = 0
    Hessian[:, Radii == 0] = 0
    Remainder = 0.5 * float(Motion @ Hessian @ Motion) + 1e-9
    Low = (Corners - Error).min(axis=0) - Remainder
    High = (Corners + Error).max(axis=0) + Remainder
    return Rotation @ ((Low + High) / 2) + Origin, Rotation, (High - Low) / 2
