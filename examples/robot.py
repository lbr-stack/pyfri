import optas


def load_robot(lbr_med_num, time_derivs):
    xacro_file_name = f"robots/med{lbr_med_num}.urdf.xacro"
    robot = optas.RobotModel(xacro_filename=xacro_file_name, time_derivs=time_derivs)
    return robot
