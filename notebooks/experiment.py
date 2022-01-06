import physics_mujoco
import time
mjcf_file_path = '../resources/ur5e/ur5e_robot.xml'
mj_model = physics_mujoco.loadModelXML(mjcf_file_path)
mj_data = mj_model.makeData()
render = physics_mujoco.Render(mj_model, mj_data)
time.sleep(10)