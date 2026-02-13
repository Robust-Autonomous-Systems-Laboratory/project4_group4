import math as m

def convertEulerToQuaternion(roll,pitch,yaw)-> tuple[float,float,float,float]:
    """
    Helper function to convert Euler angles to a Quaternion
    based on code from
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
    This could probably be done more elegantly with a service call, but the instructions state only one python file

    :param roll: Rotation about the X axis
    :param pitch: Rotation about the Y axis
    :param yaw: Rotation about the Z axis
    """
    cr = m.cos(roll * 0.5)
    sr = m.sin(roll * 0.5)
    cp = m.cos(pitch * 0.5)
    sp = m.sin(pitch * 0.5)
    cy = m.cos(yaw * 0.5)
    sy = m.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w,x,y,z