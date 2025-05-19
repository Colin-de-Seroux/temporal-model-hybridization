# Roadmap

## Objectif du projet

Le projet vise à explorer l’hybridation de modèles déductifs (issus du savoir humain) et inductifs (issus de l’analyse automatique des données collectées) dans le contexte des systèmes cyber-physiques (CPS), en particulier sur les aspects temporels avec une prise en compte de leur incertitude.

Nous nous concentrons sur les comportements temporels dans ROS 2, middleware utilisé dans les systèmes cyber-physiques, comme les véhicules définis par logiciel. Ce middleware rend difficile la prédiction des comportements d’exécution, ce qui crée un écart entre les spécifications définies par le développeur et le comportement observé à l’exécution.

L’objectif de ce PIR est donc de développer une meilleure compréhension des interactions entre la spécification des comportements temporels et l’exécution réelle des systèmes embarqués dans un environnement dynamique et incertain.

Un langage dédié (DSL) permettra d’exprimer les comportements temporels attendus ainsi que les différents composants ROS2 et leurs configurations (définition de dérive comportementale) à générer.

Pour la partie inductive, nous devons collecter des données qui seraient aussi fiables que possible en exécutant différentes configurations ROS2 (simulations). Ces données permettront ensuite de réaliser des analyses statistiques et d’entraîner un modèle d’IA.

Les caractéristiques temporelles extraites devront être informatives et réinjectées dans le DSL, afin de fournir à l’utilisateur des prédictions sur les comportements temporels d’exécution des éléments ROS 2.

## Bilan sur ce qui a été réalisé

- Création de packages simples à la main avec `ROS2`
- Création de configurations `Docker` pour un lancement simplifié
- Limitation des ressources utilisées par `Docker`
- Recherche des différents modèles d’`IA` possibles pour une architecture avec des graphes
- Écriture des logs dans la `RAM` durant l’exécution, puis sauvegarde dans des fichiers situés dans un volume spécifique à la fin de l’exécution du programme (attention à ne pas trop en demander sous peine de perte de performances)
- Création de l’architecture du `DSL` avec `Langium`
  - Création du package à partir du modèle passé en paramètre (fichier `.rosaml`)
  - Création des différents nœuds
  - Création d’un fichier `launch` pour lancer tous les nœuds
  - Création du `Dockerfile` et du morceau de code à inclure dans le `docker-compose`
  - Création du graphe au format JSON à partir du modèle (en cours d’amélioration)
- Création d’un microservice pour lire les fichiers de logs et les enregistrer dans la base de données

## Descriptif des deux livrables intermédiaires et du plan d’avancement

### Premier livrable intermédiaire

- Première version du DSL fonctionnel
- Micro-service pour enregistrer les logs

### Deuxième livrable intermédiaire

- Seconde version du DSL
- Première version de l'IA

### Rendu final

- Un DSL fonctionnel
- Une version de l'IA
- Des exemples de worflows
- Une documentation sur les limitations du projet
- Une liste des pistes d'améliorations possibles

## Bibliographie

- [Documentation officielle ROS 2](https://docs.ros.org/en/rolling/index.html)

- [Demos ROS 2](https://github.com/ros2/demos/tree/jazzy)

- [Documentation sur les algorithmes de Machine Learning pour la prévision des séries temporelles](https://medium.com/@ideaacademy/les-algorithmes-du-machine-learning-pour-la-pr%C3%A9vision-des-s%C3%A9ries-temporelles-partie-i-2b75abae4087)

- [Documentation Langium](https://github.com/eclipse-langium/langium)

- [Documentation docker](https://docs.docker.com)

- [Docker tmpfs](https://docs.docker.com/engine/storage/tmpfs)

## Organisation du travail en équipe

- [Slack avec les encadrants](https://app.slack.com/client/TMW14CTRD/C08RKFX7S2C)

- [Discord entre nous](https://discord.com)

- [Lien du Github](https://github.com/Colin-de-Seroux/temporal-model-hybridization)

- [Lien du projet Github (kanban)](https://github.com/users/Colin-de-Seroux/projects/2)

- [Lien vers les issues](https://github.com/Colin-de-Seroux/temporal-model-hybridization/issues)
