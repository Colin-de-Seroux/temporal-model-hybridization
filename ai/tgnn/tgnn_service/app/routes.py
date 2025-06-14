from flask import Blueprint, request, jsonify
from app.model_loader import load_model_and_scalers
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from tgnn_project.tgnn_project.data_loader import preprocess_ast_data

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

        # pred = predict(model, df, feature_scaler, target_scaler)

        return jsonify({df})
    except Exception as e:
        return jsonify({"error": str(e)}), 500
