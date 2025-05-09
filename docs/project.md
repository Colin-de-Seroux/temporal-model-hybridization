[talker Hello World github file](https://github.com/ros2/demos/blob/jazzy/demo_nodes_cpp/src/topics/talker.cpp)

## modèles d'IA possibles :
- GNN pour encoder le graphe à chaque instant,
- LSTM ou Transformer pour capturer l'évolution de ces embeddings temporellement.

-> GNN peuvent prendre en compte les séries temporelles, mais pas seuls. Il faut :

    - Soit enrichir les nœuds avec des caractéristiques temporelles (statistiques, tendances),
    - Soit utiliser des architectures spatio-temporelles comme T-GNN ou ST-GNN.

**GNN** graph qui pourrait contenir *plusieurs données* par *Node* ou *Edge*

**GAT** sont similaires aux GNN mais utilisent des mécanismes d'attention pour déterminer l'*importance relative* des *voisins* dans le graphe

RNN, LSTM, GRU, Modèles ARIMA 

(Documentation sur les algorithmes de Machine Learning pour la prévision des séries temporelles)[https://medium.com/@ideaacademy/les-algorithmes-du-machine-learning-pour-la-pr%C3%A9vision-des-s%C3%A9ries-temporelles-partie-i-2b75abae4087]

## Récupération des ressources utilisés

CPU/RAM : 
- psutil en python,
- prometheus metrics (node exporter...)

## Récupérer delta de temps d'exécution (prévu calculé VS Réelement exécuté)
header.stamp

## Récupérer des fréquences
ros2 topic hz/topic

## Récupérer graph ROS2
rqt_graph
