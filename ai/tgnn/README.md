# Befure using tgnn project 

placed in /tgnn :

1. create .venv

2. activate it :
```bash
.\.venv\Scripts\activate
```
[pytorch Start Locally](https://pytorch.org/get-started/locally/)

Install dependencies : 
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```
```bash
pip install -r requirements.txt

```

setup.sh has all the dependancies to install

# Training
## Description
Input : 
- csv with datas froms simulations

Output : 
- save the model trained in save/model.pth
- save/feature_scaler.pkl
- save/target_scaler.pkl

## Launch tgnn_project
```bash
cd tgnn_project
python .\__main__.py
```

# Prediction
## Description
### Flask API

Input :
- save/model.pth
- save/feature_scaler.pkl
- save/target_scaler.pkl

Routes :
- `/receive-ast` : receive in tabular format the informations of `.aml` configuration 
## Launch tgnn_service
```bash
cd tgnn_service
python .\run.py
```
