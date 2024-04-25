import onnx
from onnxsim import simplify
 
ONNX_MODEL_PATH = r"D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\tennis_c3_e.onnx"
ONNX_SIM_MODEL_PATH = r"D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\tennis_c3_e_sim.onnx"
 
if __name__ == "__main__":
    onnx_model = onnx.load(ONNX_MODEL_PATH)
    onnx_sim_model, check = simplify(onnx_model)
    assert check, "Simplified ONNX model could not be validated"
    onnx.save(onnx_sim_model, ONNX_SIM_MODEL_PATH)
    print('ONNX file simplified!')