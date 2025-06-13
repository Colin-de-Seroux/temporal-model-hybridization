from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/predict', methods=['POST'])
def predict():
    data = request.get_json()
    return jsonify({'execution_time': 123.45})

@app.route('/receive-ast', methods=['POST'])
def receive_ast():
    data = request.get_json()
    print("Received AST:")
    print(data)  
    return jsonify({"status": "AST received"}), 200

if __name__ == '__main__':
    app.run(port=5000)
