import traceback
from flask import Blueprint, request, jsonify
from app.model_loader import load_model_and_scalers
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from tgnn_project.tgnn_project.preprocess import preprocess_ast_data
from tgnn_project.tgnn_project.dataset import create_dataset
from tgnn_project.tgnn_project.predict import predict

main_blueprint = Blueprint('main', __name__)

model, feature_scaler, target_scaler = load_model_and_scalers()

@main_blueprint.route('/predict', methods=['POST'])
def predict_endpoint():
    data = request.get_json()
    return jsonify({'execution_time': 123.45})

@main_blueprint.route('/receive-ast', methods=['POST'])
def receive_ast():
    data = request.get_json()
    try:
        df = preprocess_ast_data(data)
        dataset = create_dataset(df, is_prediction=True)
        predictions = predict(model, dataset, feature_scaler, target_scaler)
        print("Received AST data:", data)
        print("Predictions:", predictions)

        return jsonify(predictions)
    except Exception as e:
        print("Exception in /receive-ast:", e)
        traceback.print_exc() 
        return jsonify({"error": str(e)}), 500
