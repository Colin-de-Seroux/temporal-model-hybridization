from flask import Flask, request, jsonify

import torch
import joblib
import pandas as pd

from tgnn_project.model import GConvGRUModel
from tgnn_project.train import predict

from tgnn_project.data_loader import preprocess_ast_data 

app = Flask(__name__)

model = GConvGRUModel(in_channels=6, out_channels=4)
model.load_state_dict(torch.load("save/model.pth"))
model.eval()


feature_scaler = joblib.load("save/feature_scaler.pkl")
target_scaler = joblib.load("save/target_scaler.pkl")

print("✅ Model and scalers loaded.")

@app.route('/predict', methods=['POST'])
def predict():
    data = request.get_json()
    return jsonify({'execution_time': 123.45})

@app.route('/receive-ast', methods=['POST'])
def receive_ast():
    data = request.get_json()
    print("Received AST:")
    print(data)  
    try:
        df = preprocess_ast_data(data)

        # pred_values = predict(model, df, feature_scaler, target_scaler)

        return jsonify({df})
    except Exception as e:
        print("❌ Error during prediction:", e)
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(port=5000)
