# Before using tgnn

Placed in `/tgnn` :

1. Create `.venv`

2. Activate it :

```bash
.\.venv\Scripts\activate
```

3. Run `nvcc --version` and get the good cuda version on pytorch for your version :

```bash
nvcc --version
```

[Pytorch website](https://pytorch.org/get-started/locally/)

4. Install dependencies :

a. Manually :

```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

```bash
pip install -r requirements.txt
```

b. With `.sh` :

Setup.sh has all the dependancies to install (change cuda version before).

```bash
./setup.sh
```

## Training

### Description

Input :

- csv with datas froms simulations

Output :

- save the model trained in `save/model.pth`
- `save/feature_scaler.pkl`
- `save/target_scaler.pkl`

### Launch tgnn_project

```bash
cd tgnn_project
python .\__main__.py
```

## Prediction

### Description

#### Flask API

Input :

- `save/model.pth`
- `save/feature_scaler.pkl`
- `save/target_scaler.pkl`

Routes :

- `/receive-ast` : receive in tabular format the informations of `.aml` configuration

### Launch tgnn_service

```bash
cd tgnn_service
python .\run.py
```
