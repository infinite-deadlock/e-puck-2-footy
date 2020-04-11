# e-puck-2-footy
Semestrial mini-project

# Verification de la lecture de pixel
+ Correction de la transmission d'informations
+ Création d'un porgamme indépendant d'analyser
+ Etude des courbes par interprétation sur graphes
Il ne reste plus que l'algorithme de détection à implémenter pour pouvroir correctement localiser les balles

# Rotation de détection
+ Systèmes de rotation opérationnel
+ Prise d'images possible de manière correcte, mais il manque l'arène
Il est possible d'acquérir les images de manière statique et d'en visionner. Un dossier en continent des échantillons sous un forme exotique.
Il reste le traitement d'images à faire pour que la phase initiale soit complète.

# Etude brève des images
+ Document d'étude sur la manière d'analyser les images
+ Plusieurs images exemples
+ Ajout de la fonctionnalité d'envoyer des images vers l'ordinateur, hors exemple du cours
Il est possible d'acquérir les images de manière statique et d'en visionner. Il faut maintenant étudier comment les traiter, après avoir pris quelques échantillons d'une vue en 360°.


# Première acquisition d'image
+ Lecture d'une image en nuance de rouge
+ Essais de détection de la balle et du but rouge
Détecter en nuance de rouge n'est pas possible, ou les objets cibles sont trop réfléchissants (balle et LEGO).
Le bruit est du même ordre de grandeurs que les objets. Essayer avec d'autres nuances de couleurs, voire les combiner.
Un premier debug serait d'acquérir une image complète et de la traiter en local pour faire les expérimentations.

# Configuration projet
+ Ajout des bases du projet à partir de TP4
+ Création des dépendances
+ Configuration de Debug
+ Création des premiers fichiers: main.c et debug.c
+ Test et implémentation d'un simple "Hello World"
Le projet est opérationnel et prêt à fonctionner