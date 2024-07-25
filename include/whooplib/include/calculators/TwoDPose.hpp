/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       TwoDPose.hpp                                              */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Two-Dimensional CFrames                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef TWODPOSE_HPP
#define TWODPOSE_HPP

#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>

/**
 * Represents a 2D pose with position and orientation in a Cartesian coordinate system.
 * Yaw is in radians, counter-clockwise.
 * +x is right-face direction, +y is front-face direction
 * The rest of the standardization is up to the end-user to decide, 
 * but is highly recommended ro stick to meters.
 *
 * If you know CFrames in Roblox, this library should be very easy to understand:
 * https://create.roblox.com/docs/reference/engine/datatypes/CFrame
 */
class TwoDPose
{
public:
    double x, y, yaw; // X and Y positions, and yaw (orientation) in radians.

    /**
     * Constructs a TwoDPose object representing a position and orientation in 2D space.
     * @param x The x-coordinate of the pose, where positive values indicate rightward movement.
     * @param y The y-coordinate of the pose, where positive values indicate forward movement.
     * @param yaw The orientation of the pose in radians, where positive values indicate counter-clockwise rotation.
     */
    TwoDPose(double x = 0, double y = 0, double yaw = 0);

    // Overloaded * operator to combine two poses
    // Think of it as Roblox's CF = CFrame1 * CFrame2
    /**
     * Combines this pose with another pose using matrix multiplication semantics.
     * Equivalent to applying the transformation of the other pose to this pose.
     * @param other Another TwoDPose to combine with this pose.
     * @return A new TwoDPose representing the combined transformation.
     */
    TwoDPose operator*(const TwoDPose &other) const;

    // This allows for *= as well
    /**
     * Applies and assigns the transformation of another pose to this pose.
     * @param other Another TwoDPose whose transformation is applied to this pose.
     * @return A reference to this pose after the transformation.
     */
    TwoDPose &operator*=(const TwoDPose &other);

    // Returns a TwoDPose that represents the delta change applied to the *=, relative to the first TwoDPose
    /**
     * Calculates the transformation required to go from this pose to another pose.
     * This can be thought of as the relative movement and rotation from this pose to the specified pose.
     * @param other The pose to which the transformation is calculated.
     * @return A TwoDPose representing the required transformation.
     */
    TwoDPose global_xy_delta_only(const TwoDPose &other) const;

    // Method to compute the relative pose
    // Think of it as Roblox's CFrame1:ToObjectSpace(CFrame2)
    // in which it returns the pose of CFrame2 relative to CFrame1
    // But in this case, TwoDPose other relative to TwoDPose self
    /**
     * Transforms another pose into the coordinate space of this pose.
     * This is analogous to calculating the pose of an object relative to a reference frame defined by this pose.
     * @param other The pose to transform into this pose's coordinate space.
     * @return A new TwoDPose representing the pose relative to this pose.
     */
    TwoDPose toObjectSpace(const TwoDPose &other) const;

    // Method to retreive world space from object space
    // Example of getting object space:
    // TwoDPose b_in_a_object_space = a.toObjectSpace(b);
    // You can then recover b back with:
    // TwoDPose b_recovered = a.toWorldSpace(b_in_a_object_space);
    /**
     * Transforms a pose from the local coordinate space of this pose back into the global coordinate space.
     * This is the inverse operation of toObjectSpace, effectively recalculating the global coordinates of a pose
     * that has been transformed into this pose's local space. It re-applies the global positioning and orientation
     * of this pose to the local pose.
     * @param other The pose in this object's local coordinate space to transform back to global space.
     * @return A new TwoDPose representing the pose in global coordinates.
     */
    TwoDPose toWorldSpace(const TwoDPose &other) const;

    // Unary negation operator
    TwoDPose operator-() const;

    // Method to compute relative pose of just given x y and yaw.
    // This is useful for tare.
    /**
     * Transforms given position and orientation coordinates into the coordinate space of this pose.
     * Useful for recalibrating or 'taring' sensors relative to a current pose.
     * @param x X-coordinate to transform.
     * @param y Y-coordinate to transform.
     * @param yaw Orientation in radians to transform.
     * @return A new TwoDPose representing the transformed coordinates.
     */
    TwoDPose toObjectSpace(double x, double y, double yaw) const;

    /**
     * Returns a pose with the same x and y, but with the yaw turned to face the designated x and y
     * @param target_x the x position to look at
     * @param target_y the y position to look at
     */
    TwoDPose lookAt(double target_x, double target_y);

    /**
     * Returns a string representation of the pose with the coordinates rounded to a specified number of decimal places.
     * If decimal_places is -1, no rounding is applied.
     * @param decimal_places The number of decimal places to round to, or -1 for no rounding.
     * @return A string in the format "x y yaw".
     */
    std::string to_string(int decimal_places = 4);

    /**
     * Returns a string representation of the pose with the coordinates rounded to a specified number of decimal places.
     * This variant follows pose standard for the T265
     * If decimal_places is -1, no rounding is applied.
     * @param decimal_places The number of decimal places to round to, or -1 for no rounding.
     * @return A string in the format "x y yaw".
     */
    std::string to_realsense_string(int decimal_places = 4);
};

#endif // TWODPOSE_HPP
