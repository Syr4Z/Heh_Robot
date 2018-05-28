#
# Auteurs du code :
#
# Nom : Ronvaux
# Prenom : Sam
#
# Nom : Stillemant
# Prenom Nicolas
#
# Nom : Hoekman
# Prenom : Christopher

#Importation des librairies
import piconzero as pz, time
import RPi.GPIO as GP
import RPi.GPIO as GPIO
import sys
import tty
import termios
import os
import logging
from logging.handlers import RotatingFileHandler

#Mode DEBUG
debug = "false"

#Calibrage des angles
ANGLE = dict()

ANGLE["gauche"] = 1.4
ANGLE["droite"] = 1.10

#Definition des PINs
PIN = dict()

#Pin du sonar avant
PIN["TrigPinAvant"] = 04
PIN["EchoPinAvant"] = 17

#Pin du sonar de gauche
PIN["TrigPinGauche"] = 27
PIN["EchoPinGauche"] = 22

#Systeme de logs
# creation de l objet logger qui va nous servir a ecrire dans les logs
logger = logging.getLogger()
# on met le niveau du logger a DEBUG, comme ca il ecrit tout
logger.setLevel(logging.DEBUG)

# creation d un formateur qui va ajouter le temps, le niveau
# de chaque message quand on ecrira un message dans le log
formatter = logging.Formatter('%(asctime)s :: %(levelname)s :: %(message)s')

# creation d un handler qui va rediriger une ecriture du log vers
# un fichier en mode 'append', avec un backup et une taille max de 1Mo
file_handler = RotatingFileHandler('robot.log', 'a', 1000000, 1)

# on lui met le niveau sur DEBUG, on lui dit qu'il doit utiliser le formateur
# cree precedement et on ajoute ce handler au logger
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# creation d un second handler qui va rediriger chaque ecriture de log
# sur la console si le mode DEBUG est actif
if debug == "true" :
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.DEBUG)
    logger.addHandler(stream_handler)

#Fonction d arret et de demarrage de la camera
def camera(action):

    if action == 1 :
        print "##################################################"
        print "# Demarrage de la camera pour le controle manuel #"
        os.system('sudo systemctl start motion')
        print "# La camera a bien ete demarre                   #"
        print "##################################################"
        logger.info('La camera a ete demarre')

    elif action == 0 :
        print "################################"
        print "# Arret de la camera           #"
        os.system('sudo systemctl stop motion')
        print "# La camera a bien ete  arrete #"
        print "################################"
        logger.info('La camera a ete arretee')

    else :
        logger.warning('Une erreur dans le module de camera s est produite')

def tuto():
    print "Vous pouvez controler le robot a l aide des fleches directionnel ou de la manette."
    print "Appuyer sur le bouton START\n ou sur la touche espace pour que le robot sorte du labyrinthe."
    print "En mode navigation libre vous pouvez augmenter la vitesse avec les gachette."
    logger.info('Le tuto a ete affiche')

#Fonction de gestion du robot grace a la manette et au clavier
def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)

def Sonar(position):

    #Variable global
    global PIN
    #Definition des variables rapides
    GPIO_TRIGGER = PIN["TrigPin" + position]
    GPIO_ECHO = PIN["EchoPin" + position]

    #Init de la variable distance
    distance = 300
    #Choix du mode BCM pour les GPIO.
    GPIO.setmode(GPIO.BCM)

    #Mise en Input du Echo et en Output pour Trig
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)  # Trigger
    GPIO.setup(GPIO_ECHO, GPIO.IN)  # Echo

    while distance >= 300:
        #Mise a 0 du Trig
        GPIO.output(GPIO_TRIGGER, False)

        #Delai pour la bonne reception
        time.sleep(0.0001)

        #Envoi du signal
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        start = time.time()
        while GPIO.input(GPIO_ECHO) == 0:
            start = time.time()
        while GPIO.input(GPIO_ECHO) == 1:
            stop = time.time()

        #Calcul de la distance en fonction de la longueur d onde
        elapsed = stop - start

        #On multiplie la valeur de temps par le nombre de cm que le son parcours par seconde (34000)
        distance = elapsed * 34000

        # La valeur doit maintenant etre divise par deux car l onde a parcouru un aller et un retour
        distance = distance / 2

    logger.info('Distance a ' + position + ' : ' + str(distance) + ' cm')
    # Reset GPIO
    GPIO.cleanup()

    # Renvoi de la distance
    return distance

def mru():
    distance1 = Sonar(position="Avant")/100
    time.sleep(0.1)
    distance2 = Sonar(position="Avant")/100
    mru = (distance1 - distance2)/0.1
    mru = mru * 100
    return mru

def ControleRobot(action):
    global ANGLE
    speed = 85
    if action == "avance":
        pz.reverse(speed)

    elif action == "recule":
        pz.forward(speed)
        time.sleep(0.1)

    elif action == "droite":
        pz.stop()
        time.sleep(1)
        pz.spinLeft(95)
        time.sleep(ANGLE["droite"])

    elif action == "gauche":
        time.sleep(0.1)
        pz.stop()
        time.sleep(1)
        pz.spinRight(95)
        time.sleep(ANGLE["gauche"])
        pz.reverse(speed)
        time.sleep(0.3)

    elif action == "arrete":
        pz.stop()

    else:
        logger.warning('Une erreur dans les commandes de controle du robot s est produite !')

def labyrinthe():
    logger.info('L utilisateur demarre le mode labyrinthe')
    CulDeSac = 0
    ActionPrecedente = 0
    while Sonar(position="Avant") > 10 and Sonar(position="Gauche") > 200:
        ControleRobot(action="avance")
    try:
        while True:
            DistanceAvant = Sonar(position="Avant")
            DistanceGauche = Sonar(position="Gauche")

            if DistanceGauche > 30:
                if ActionPrecedente >= 2:
                    CulDeSac =+ 1
                ActionPrecedente = 0
                time.sleep(0.1)
                ControleRobot(action="gauche")
                time.sleep(0.5)
                logger.info('Le robot va a gauche')

            elif DistanceAvant <= 7 and DistanceGauche <= 30:
                ActionPrecedente = 1
                ControleRobot(action="droite")
                logger.info('Le robot va a droite')

            elif DistanceAvant > 7:
                if ActionPrecedente >= 2:
                    CulDeSac =+ 1
                ActionPrecedente = 0
                ControleRobot(action="avance")
                logger.info('Le robot va tout droit')

            elif DistanceAvant < 7:
                if ActionPrecedente >= 2:
                    CulDeSac =+ 1
                ActionPrecedente = 0
                ControlRobot(action="recule")
                logger.info('Le robot recule')
            else:
                ControleRobot("arrete")
                logger.warning('Une erreur dans le module de resolution du labyrinthe s est produite !')
                break
    except KeyboardInterrupt:
        print "Le robot a rencontre " + CulDeSac + "cul de sac durant son parcours"
        print

    finally:
        print "Arret du mode labyrinthe"
        ControleRobot(action="arrete")
        camera(action=1)
        time.sleep(5)
        os.system('clear')
        tuto()
        logger.info('L utilisateur a quitte le mode labyrinthe')

#MAIN
logger.info('L utilisateur est passe en mode manuel')
camera(action=1)
tuto()
speed=65

pz.init() #Initialise l utilisation des fonctions PZ contenue dans la librairie PICONZERO

# Boucle principale
try:
    while True:
        keyp = readkey()

        #Permet de faire avancer le robot tout droit a l aide la manette et les touches.
        if keyp == 'z' or ord(keyp) == 16:
            pz.reverse(speed)
            logger.info('Le robot va tout droit.')
            print 'Avance a une vitesse de ' + str(mru()) + 'cm/s.', speed

        #Permet de faire reculer le robot a l aide de la manette et les touches.
        elif keyp == 's' or ord(keyp) == 17:
            pz.forward(speed)
            print 'Recule a une vitesse de ' + str(mru()) + 'cm/s.', speed

        #Permet de faire tourner le robot vers la droite a l aide de la manette et les touches.
        elif keyp == 'd' or ord(keyp) == 19:
            pz.spinRight(75)
            logger.info('Le robot a tourne a droite.')
            print 'Tourne a droite', speed

        #Permet de faire tourner le robot vers la gauche a l aide de la manette et des touches.
        elif keyp == 'q' or ord(keyp) == 18:
            pz.spinLeft(75)
            logger.info('Le robot a tourne a gauche.')
            print 'Tourne a gauche', speed

        #Permet d augmenter la vitesse du robot a l aide de la manette et des touches.
        elif keyp == '+' or keyp == '>':
            speed = min(100, speed+10)
            pz.stop()
            pz.reverse(speed)
            logger.info('Le robot augmente la vitesse')
            print 'Augmente vitesse', speed

        #Permet de diminuer la vitesse du robot a l aide de la manette et des touches.
        elif keyp == '-' or keyp == '<':
            speed = max (0, speed-10)
            pz.stop()
            pz.reverse(speed)
            logger.info('Le robot diminue la vitesse')
            print 'Diminue vitesse', speed

        elif keyp == 'x':
            pz.stop()
            logger.info('Le robot s arrete.')

        # Permet de demarrer la fonction de resolution de labyrinthe.
        elif keyp == ' ' :
            pz.stop()
            print 'Demarre le mode labyrinthe'
            camera(action=0)
            labyrinthe()

        # Permet de quitter le programme
        elif ord(keyp) == 3:
            pz.stop()
            break

# Permet de gerer la sortie de programme de maniere propre.
except KeyboardInterrupt:
    print

# permet de vider les emplacements reserves
finally:
    print "Fin du programme"
    logger.info('##### Fin du Programme #####')
    camera(action=0)
    pz.cleanup()