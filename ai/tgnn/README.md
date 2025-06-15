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

> [!Warning]
> If you have an error like this on windows:

```sh
error: Microsoft Visual C++ 14.0 or greater is required. Get it with "Microsoft C++ Build Tools": https://visualstudio.microsoft.com/visual-cpp-build-tools/
      [end of output]

  note: This error originates from a subprocess, and is likely not a problem with pip.
  ERROR: Failed building wheel for torch_sparse
  Running setup.py clean for torch_sparse
Failed to build torch_scatter torch_sparse
ERROR: Failed to build installable wheels for some pyproject.toml based projects (torch_scatter, torch_sparse)
```

1. Go download [Visual Studio Setup](https://visualstudio.microsoft.com/visual-cpp-build-tools/).

2. Click on `C++ build tools`.

3. Click on `Download`.

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
