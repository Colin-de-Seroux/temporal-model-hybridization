#!/bin/bash
# sudo apt install python3.10-venv
# python3 -m .venv venv
# source venv/bin/activate

pip install --upgrade pip

pip install torch==2.7.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install torch-scatter torch-sparse -f https://data.pyg.org/whl/torch-2.7.0+cu118.html

pip install torch-geometric
pip install torch-geometric-temporal

pip install numpy
pip install pandas
pip install Flask
pip install Flask-Cors
pip install scikit-learn
pip install matplotlib

echo "Installation terminée avec succès !"
