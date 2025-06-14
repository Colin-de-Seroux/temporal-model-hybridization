#!/bin/bash
# sudo apt install python3.10-venv
# python3 -m .venv venv
# source venv/bin/activate

pip install --upgrade pip

pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

pip install torch-geometric

pip install torch-geometric-temporal

pip install numpy
pip install pandas
pip install Flask
pip install Flask-Cors
pip install scikit-learn
pip install matplotlib


echo "Installation terminée avec succès !"
