#!/usr/bin/python

import rospy
import math

import random
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

#Parametre permettant de fixer la taille de l'endroit a explorer la precision du tableau d'exploration et le temps de la recherche en aveugle avant la recherche intelligente
global tailleMapX
global tailleMapY
global debutMapX
global debutMapY
global precision
global ratio
global debutLigne
global debutColonne
global nbrLigne
global nbrColonne
global moitierLigne
global moitierColonne
global tempsRechercheAveugle

# Les quatres variables suivantes permettent de definir la taille de la carte a explorer on pourrait imaginer les faire changer en fonction de capteur sur des robots.
tailleMapX = 11
tailleMapY = 11
debutMapX = 0
debutMapY = 0
#Precision de 0.1 pour une exploration vraiment complete de la carte mais vraiment longue ! je preconise 0.2 et 0.5 pour une rapide
precision = 0.2
ratio = 1/precision
#Valeur pour creer le tableau
nbrLigne = int(tailleMapY * ratio)
nbrColonne = int(tailleMapX * ratio)
debutLigne = int(debutMapY * ratio)
debutColonne = int(debutMapX * ratio)
moitierLigne = nbrLigne / 2
moitierColonne = nbrColonne / 2
#Temps entre la recherche rapide mais avaeugle (j'avance jusqu'a un mur je tourne et je recommence) et la recherche intelligente (on cherche un point non decouvert et on y va)
tempsRechercheAveugle = 300

def dist(x1,y1,x2,y2):
	# Retourne la distance entre deux points
	dx=x1-x2
	dy=y1-y2
	return math.sqrt(dx*dx+dy*dy)


def tournerDroiteOuGauche(x,y,theta):
	#Permet de savoir l'orientation de colision avec le mur pour savoir dans quel direction la tortue doit tourner
	tournerGauche = 0
	tournerDroite = 0
	if theta < 0:
		theta=theta+6.28
	if (theta<=1.57 and theta>=0) and x>=tailleMapX:
		tournerGauche=1
	elif (theta<=1.57 and theta>=0) and y>=tailleMapY:
		tournerDroite =1
	elif (theta<=3.14 and theta>=1.57) and y>=tailleMapY:
		tournerGauche=1
	elif (theta<=3.14 and theta>=1.57) and x<=debutMapX:
		tournerDroite =1
	elif (theta<4.71 and theta>3.14) and x<=debutMapX:
		tournerGauche=1
	elif (theta<4.71 and theta>3.14) and y<=debutMapY:
		tournerDroite =1
	elif (theta<6.28 and theta>4.71) and y<=debutMapY:
		tournerGauche=1
	elif (theta<6.28 and theta>4.71) and x>=tailleMapX:
		tournerDroite =1
	if tournerGauche==1:
		return 1
	elif tournerDroite ==1:
		return 2
	else:
		#~ print "theta = ",theta,"x = ",x,"y = ",y
		return 0
		
def valeurDroiteOuGauche(droiteOuGaucheValeur):
	#Retourne une valeur aleatoire entre 2.2 et 4.2 
	if droiteOuGaucheValeur==1:
		return 2.2+(random.random()*2)
	elif droiteOuGaucheValeur==2:
		return -2.2-(random.random()*2)
	else:
		return 0

def rencontrerMur(tempsSurLeMur,numMsg,numPublisher,droiteOuGauche):
	numMsg=Twist()
	#Tant que le temps de rotation est pas ecoule on fait tourner la tortue
	if (time.time()-tempsSurLeMur)<0.60:
		valeurRetourner=0
		numMsg.angular.z=valeurDroiteOuGauche(droiteOuGauche)
		numPublisher(numMsg)
	else:
		valeurRetourner=1
	return valeurRetourner
	
def bougerTortue(self,x,y,theta,numPublisher,tourner,dansMur,tempsMur,avancer,droiteOuGauche,repartir,colision,tempsColision,angleCo,map,tortueEnHaut,tortueADroite,cibleX,cibleY,typeRecherche,nvlCible,parcourSansMap,numTortue):
	#~ print "tortue numero : ",numTortue
	#~ print "x = ",x,"y = ",y
	msg=Twist()
	tTheta=0
	d=0
	dTheta=0
	milieu = 0
	tortueStopper = 0
	tortueStopper2 = 0
	tortueStopper3 = 0
	damping = 1000
	commencerAvecMap = 0
	compteurPointRestant = 0
	if time.time()-parcourSansMap > tempsRechercheAveugle:
		commencerAvecMap = 1
	map=noterPointDansMap(x,y,map,self)
	for i in range(debutLigne,nbrLigne+1,1):
		for j in range(debutColonne,nbrColonne+1,1):
			if map[i][j]==0:
				compteurPointRestant += 1
	if chercherPointNonAtteint(map) == 1:
		#~ print "il reste encore des points non decouverts"
		#~ print "cilbeX = ",cibleX,"cibleY = ",cibleY
		testCo=testColision(self,numTortue)
		if testCo==0 and repartir==1:
			#~ print "pas de colision et on repart tourner = ",tourner, "dans mur = ",dansMur
			if (x >=tailleMapX or x <=debutMapX or y<=debutMapY or y>=tailleMapY) and tourner==0:
				#~ print "dans condition toucher mur"
				if dansMur==0:
					droiteOuGauche=tournerDroiteOuGauche(x,y,theta)
					if droiteOuGauche == 0:
						#~ print "tourner mais encore dans mur"
						msg.linear.x = 5
						numPublisher(msg)
					else :
						tempsMur=time.time()
						dansMur=1
						avancer=0
				if tourner==1:
					avancer=1
					nvlCible = 1
					msg.linear.x = 5
					numPublisher(msg)
				elif tourner == 0:
					#~ print "temps ecoule depuis dansmur = ",time.time()-tempsMur
					tourner=rencontrerMur(tempsMur,msg,numPublisher,droiteOuGauche)
			elif avancer==1:
				if (x <tailleMapX and x >debutMapX and y>debutMapY and y<tailleMapY): # Pour eviter que la tortue se bloque entre les deux conditions
					dansMur=0
					tourner=0
				if commencerAvecMap == 1:
					print "Point restant = ",compteurPointRestant
					if map[int(round(cibleY,1) * ratio)][int(round(cibleX,1) * ratio)] == 1 or nvlCible == 1:
						(cibleX,cibleY,typeRecherche)=chercherPointDansMap(x,y,theta,map,typeRecherche)
						nvlCible = 0
						print "cibleX = ",cibleX, "cibleY = ",cibleY
					d=dist(x,y,cibleX,cibleY)
					tTheta=math.atan2(cibleY-y,cibleX-x)
					dTheta=tTheta-theta
					if dTheta < -1:
						dTheta += 6.28
					#~ print "dTheta = ",dTheta
					if typeRecherche == 1:
						msg.angular.z=dTheta*2
						if dTheta < 2 and dTheta > -2 :
							msg.linear.x=10*d/(1+damping*abs(dTheta))
					elif typeRecherche == 2:
						msg.angular.z=dTheta*3
						if dTheta<0.001 and dTheta > -0.001 :
							msg.linear.x=10*d/(1+damping*abs(dTheta))
					else:
						print "erreur avec la recherche de cible au niveau de typeRecherche"
				else:
					if tourner == 1 and dansMur == 1:
						dansMur = 0
						tourner = 0
					#~ print "tortue avance"
					msg.linear.x = 5
				numPublisher(msg)
			else:
				#~ print "probleme avec certaine mise au point permet de faire avancer la tortue si probleme"
				#~ tourner = 0
				avancer =1
		else:
			#~ print "soit une colision soit on repart pas encore soit les deux"
			(tortueADroite,tortueEnHaut)=posTortueHautDroite(testCo,numTortue,self,tortueADroite,tortueEnHaut)
			if colision==0 or (testCo != 0 and repartir == 1):
				stopTortue(self.pub.publish)
				stopTortue(self.pub2.publish)
				stopTortue(self.pub3.publish)
				#~ print "colision"
				if testCo == 1 and numTortue == 1:
					angleCo=angleColision(self.theta,self.x,self.y,self.theta2,self.x2,self.y2)
				elif testCo == 1 and numTortue == 2:
					angleCo=angleColision(self.theta2,self.x2,self.y2,self.theta,self.x,self.y)
				elif testCo == 2 and numTortue == 1:
					angleCo=angleColision(self.theta,self.x,self.y,self.theta3,self.x3,self.y3)
				elif testCo == 2 and numTortue == 3:
					angleCo=angleColision(self.theta3,self.x3,self.y3,self.theta,self.x,self.y)
				elif testCo == 3 and numTortue == 2:
					angleCo=angleColision(self.theta2,self.x2,self.y2,self.theta3,self.x3,self.y3)
				elif testCo == 3 and numTortue == 3:
					angleCo=angleColision(self.theta3,self.x3,self.y3,self.theta2,self.x2,self.y2)
				tempsColision=time.time()
				repartir = 0
				colision=1
			if time.time()-tempsColision< 0.6:
				#~ print"Dans partie ou ya colision on reagis pendant 0.6s"
				testCoArriere=testColision(self,numTortue)
				if testCoArriere != testCo :
					#Si la tortue deja en gestion de colision touche une autre tortue 
					if numTortue == 1:
						#On l'arrete
						stopTortue(self.pub.publish)
						tortueStopper=1
					elif numTortue == 2:
						stopTortue(self.pub2.publish)
						tortueStopper2=1
					elif numTortue == 3:
						stopTortue(self.pub3.publish)
						tortueStopper3=1
					if tortueStopper == 1 or tortueStopper2 == 1 or tortueStopper3 == 1:
						#On fait tourner la tortue ver le point qui convient en fonction de sa position
						(tortueADroite,tortueEnHaut)=posTortueHautDroiteA3(testCo,numTortue,self,tortueADroite,tortueEnHaut)
						if tortueADroite ==1 and tortueEnHaut == 1:
							tTheta=math.atan2((tailleMapY-0.1)-y,(tailleMapX-0.1)-x)
						elif tortueADroite == 0 and tortueEnHaut == 0:
							tTheta=math.atan2((debutMapY+0.1)-y,(debutMapX+0.1)-x)
						elif tortueADroite == 0 and tortueEnHaut == 2:
							tTheta=math.atan2(y-y,(debutMapX+0.1)-x)
						elif tortueADroite == 1 and tortueEnHaut == 2:
							tTheta=math.atan2(y-y,(tailleMapX-0.1)-x)
						elif tortueADroite == 2 and tortueEnHaut == 0:
							tTheta=math.atan2((debutMapY+0.1)-y,x-x)
						elif tortueADroite == 2 and tortueEnHaut == 1:
							tTheta=math.atan2((tailleMapY-0.1)-y,x-x)
						elif tortueADroite == 2 and tortueEnHaut == 2:
							milieu = 1
						if milieu == 1:
							msg.angular.z = 0
							msg.linear.x = 0
						else:
							msg.angular.z=dTheta*3
							if dTheta<0.1:
								msg.linear.x = 2
				else :
					if x >=tailleMapX or x <=debutMapX or y<=debutMapY or y>=tailleMapY:
						#~ print "et on touche un mur en reculant"
						#~ print "tortue numero ",numTortue,"position haute :",tortueEnHaut,"position droite :",tortueADroite
						if x >= tailleMapX:
							if tortueEnHaut == 1:
								tTheta=math.atan2(tailleMapY-y,(tailleMapX-0.5)-x)
							else:
								tTheta=math.atan2(debutMapY-y,(tailleMapX-0.5)-x)
						if x <= debutMapX:
							if tortueEnHaut == 1:
								tTheta=math.atan2(tailleMapY-y,(debutMapX+0.5)-x)
							else:
								tTheta=math.atan2(debutMapY-y,(debutMapX+0.5)-x)
						if y <= debutMapY:
							if tortueADroite == 1:
								tTheta=math.atan2((debutMapY+0.5)-y,tailleMapX-x)
							else:
								tTheta=math.atan2((debutMapY+0.5)-y,debutMapX-x)
						if y >= tailleMapY:
							if tortueADroite == 1:
								tTheta=math.atan2((tailleMapY-0.5)-y,tailleMapX-x)
							else:
								tTheta=math.atan2((tailleMapY-0.5)-y,debutMapX-x)
						dTheta = tTheta-theta
						msg.angular.z = (dTheta)*3
						if tTheta-theta < 0.2 and dTheta > -0.2:
							msg.linear.x=3
						else :
							msg.linear.x=0
					#Si la tortue n'est pas dans un mur elle avance en fonction de sa position et de son orientation lors de la colision
					#Les valeur d'angleCo sont decris dans la fonction angleColision
					elif angleCo==1:
						#Si on regarde pas vers un mur et qu'on y est proche alors on avance sinon on recule en tournant
						if (theta < 3.14 and theta > 0 and y > tailleMapY - 1.5) or (theta < 6.18 and theta > 3.14 and y < debutMapY + 1.5) or (theta < 4.71 and theta > 1.57 and x < debutMapX + 1.5) or (theta < 1.57 and theta > 4.71 and x > tailleMapX - 1.5):
							angleCo = 4
						else:
							msg.linear.x=3
							msg.angular.z=0
					elif angleCo==2:
						msg.linear.x=-2
						msg.angular.z=0
						msg.angular.z=-0.5
					elif angleCo==3:
						msg.linear.x=-4
						msg.angular.z=2
					elif angleCo==4:
						msg.linear.x=-4
						msg.angular.z=-2
					elif angleCo==10:
						msg.angular.z = 1.5
					else:
						print"Reponse colision tortue a echouer"
					numPublisher(msg)
			else:
				#~ print "temps gestion colision ecouler"
				repartir = 1
				colision = 0
				nvlCible=1
				tourner=0
				dansMur=0
	else:
		#~ print "plus de points non decouvert"
		kill=rospy.ServiceProxy("/kill",Kill)
		if numTortue == 1:
			self.tortueVivante = 0
			self.x = -3
			self.y = -3
			tuer=kill("turtleA")
		elif numTortue == 2 :
			self.tortueVivante2 = 0
			self.x2 = -3
			self.y2 = -3
			tuer=kill("turtleBob")
		elif numTortue == 3 :
			self.tortueVivante3 = 0
			self.x3 = -3
			self.y3 = -3
			tuer=kill("turtleRobotLabo")
	return dansMur,tempsMur,droiteOuGauche,tourner,avancer,repartir,colision,tempsColision,angleCo,map,tortueEnHaut,tortueADroite,cibleX,cibleY,typeRecherche,nvlCible

def testColision(self,numTortue):
	rencontre = 0
	if numTortue==1 or numTortue==2:
		if (self.x <= self.x2+0.9 and self.x >= self.x2-0.9) and (self.y <= self.y2+0.9 and self.y >= self.y2-0.9):
			rencontre = 1
			#La ligne qui suit est pour eviter que deux tortues coller au mur ou dans un coin mais ayant un angle de relance different restent bloque
			if ((self.x <= debutMapX and self.x2 <=debutMapX) or (self.x >= tailleMapX and self.x2 >=tailleMapX) or (self.y <= debutMapY and self.y2 <=debutMapY) or (self.y >= tailleMapY and self.y2 >=tailleMapY) or (self.y <= debutMapY and self.x2 <= debutMapX) or (self.y2 <= debutMapY and self.x <= debutMapX) or (self.y >= tailleMapY and self.x2 <= debutMapX) or (self.y2 >= tailleMapY and self.x <= debutMapX) or (self.y <= debutMapY and self.x2 >= tailleMapX) or (self.y2 <= debutMapY and self.x >= tailleMapX) or (self.y >= tailleMapY and self.x2 >= tailleMapX) or (self.y2 >= tailleMapY and self.x >= tailleMapX)) and (self.theta < self.theta2-1 or self.theta > self.theta2+1):
				rencontre = 0
	if numTortue==1 or numTortue==3:
		if (self.x <= self.x3+0.9 and self.x >= self.x3-0.9) and (self.y <= self.y3+0.9 and self.y >= self.y3-0.9):
			rencontre = 2
			if ((self.x <= debutMapX and self.x3 <=debutMapX) or (self.x >= tailleMapX and self.x3 >=tailleMapX) or (self.y <= debutMapY and self.y3 <=debutMapY) or (self.y >= tailleMapY and self.y3 >=tailleMapY) or (self.y <= debutMapY and self.x3 <= debutMapX) or (self.y3 <= debutMapY and self.x <= debutMapX) or (self.y >= tailleMapY and self.x3 <= debutMapX) or (self.y3 >= tailleMapY and self.x <= debutMapX) or (self.y <= debutMapY and self.x3 >= tailleMapX) or (self.y3 <= debutMapY and self.x >= tailleMapX) or (self.y >= tailleMapY and self.x3 >= tailleMapX) or (self.y3 >= tailleMapY and self.x >= tailleMapX)) and (self.theta < self.theta3-1 or self.theta > self.theta3+1):
				rencontre = 0
	if numTortue==2 or numTortue ==3:
		if (self.x3 <= self.x2+0.9 and self.x3 >= self.x2-0.9) and (self.y3 <= self.y2+0.9 and self.y3 >= self.y2-0.9):
			rencontre = 3
			if ((self.x2 <= debutMapX and self.x3 <=debutMapX) or (self.x2 >= tailleMapX and self.x3 >=tailleMapX) or (self.y2 <= debutMapY and self.y3 <= debutMapY) or (self.y2 >= tailleMapY and self.y3 >=tailleMapY) or (self.y3 <= debutMapY and self.x2 <= debutMapX) or (self.y2 <= debutMapY and self.x3 <= debutMapX) or (self.y3 >= tailleMapY and self.x2 <= debutMapX) or (self.y2 >= tailleMapY and self.x3 <= debutMapX) or (self.y3 <= debutMapY and self.x2 >= tailleMapX) or (self.y2 <= debutMapY and self.x3 >= tailleMapX) or (self.y3 >= tailleMapY and self.x2 >= tailleMapX) or (self.y2 >= tailleMapY and self.x3 >= tailleMapX)) and (self.theta2 < self.theta3-1 or self.theta2 > self.theta3+1):
				rencontre = 0
	return rencontre
		
def stopTortue(numPublisher):
	msg=Twist()
	msg.linear.x=0
	msg.angular.z=0
	numPublisher(msg)

def angleColision(thetaTortueBouge, xTortueBouge, yTortueBouge, thetaTortueCompare, xTortueCompare, yTortueCompare):
	#~ Dans cette fonction on retourne 1 si la tortue doit juste avancer et 2 si elle doit juste reculer
	#~ 3 si elle doit reculer en tournant a droite et 4 pour reculer en tournant a gauche
	if thetaTortueBouge < 0:
		thetaTortueBouge=thetaTortueBouge+6.28
	if thetaTortueCompare <0:
		thetaTortueCompare=thetaTortueCompare+6.28
	#~ On Regarde si Les tortues vont dans la meme direction:
	#~ Vers le haut
	if ((thetaTortueBouge<=1.57 and thetaTortueBouge>=0) and (thetaTortueCompare<=1.57 and thetaTortueCompare>=0)) or ((thetaTortueBouge<=3.14 and thetaTortueBouge>=1.57) and (thetaTortueCompare<=3.14 and thetaTortueCompare>=1.57)):
		if yTortueBouge < yTortueCompare: #Si y est plus petit et qu'on va vers le haut il vaut mieux reculer
			#~ print "condition 1 "
			return 2
		elif yTortueBouge > yTortueCompare: 
			#~ print "condition 2 "
			return 1
		else:
			return 10
	#Vers le bas:
	elif ((thetaTortueBouge<=4.71 and thetaTortueBouge>=3.14) and (thetaTortueCompare<=4.71 and thetaTortueCompare>=3.14)) or ((thetaTortueBouge<=6.28 and thetaTortueBouge>=4.71) and (thetaTortueCompare<=6.28 and thetaTortueCompare>=4.71)) :
		if yTortueBouge < yTortueCompare:
			#~ print "condition 3"
			return 1
		elif yTortueBouge > yTortueCompare: 
			#~ print "condition 4"
			return 2
		else:
			return 10
	#~ On regarde si les tortues vont dans la meme direction mais avec deux sens different (On fera reculer la tortue ayant la plus petite coordone selon la direction ou elles vont)
	#droite
	elif ((thetaTortueBouge<=1.57 and thetaTortueBouge>=0) and (thetaTortueCompare<=6.28 and thetaTortueCompare>=4.71)) or ((thetaTortueBouge<=6.28 and thetaTortueBouge>=4.71) and (thetaTortueCompare<=1.57 and thetaTortueCompare>=0)) :
		if xTortueBouge < xTortueCompare:
			#~ print "condition 5"
			return 2
		elif xTortueBouge > xTortueCompare: 
			#~ print "condition 6"
			return 1
		else:
			return 10
	#gauche
	elif ((thetaTortueBouge<=3.14 and thetaTortueBouge>=1.57) and (thetaTortueCompare<=4.71 and thetaTortueCompare>=3.14)) or ((thetaTortueBouge<=4.71 and thetaTortueBouge>=3.14) and (thetaTortueCompare<=3.14 and thetaTortueCompare>=1.57)) :
		if xTortueBouge < xTortueCompare:
			#~ print "condition 7"
			return 1
		elif xTortueBouge > xTortueCompare: 
			#~ print "condition 8"
			return 2
		else:
			return 10
	#haut
	elif ((thetaTortueBouge<=1.57 and thetaTortueBouge>=0) and (thetaTortueCompare<=3.14 and thetaTortueCompare>=1.57) ) or ((thetaTortueBouge<=3.14 and thetaTortueBouge>=1.57) and (thetaTortueCompare<=1.57 and thetaTortueCompare>=0)) :
		if yTortueBouge < yTortueCompare:
			#~ print "condition 9"
			return 2
		elif yTortueBouge > yTortueCompare: 
			#~ print "condition 10"
			return 1
		else:
			return 10
	#Bas
	elif ((thetaTortueBouge<=4.71 and thetaTortueBouge>=3.14) and (thetaTortueCompare<=6.28 and thetaTortueCompare>=4.71)) or ((thetaTortueBouge<=6.28 and thetaTortueBouge>=4.71) and (thetaTortueCompare<=4.71 and thetaTortueCompare>=3.14)) :
		if yTortueBouge < yTortueCompare:
			#~ print "condition 11"
			return 1
		elif yTortueBouge > yTortueCompare: 
			#~ print "condition 12"
			return 2
		else:
			return 10
	#Sens oppose
	#diagonale haut gauche/bas droite
	elif ((thetaTortueBouge<=6.28 and thetaTortueBouge>=4.71) and (thetaTortueCompare<=3.14 and thetaTortueCompare>=1.57)) or ((thetaTortueBouge<=3.14 and thetaTortueBouge>=1.57) and (thetaTortueCompare<=6.28 and thetaTortueCompare>=4.71)) :
		if yTortueBouge < yTortueCompare:
			#~ print "condition 13"
			return 4
		elif yTortueBouge > yTortueCompare: 
			#~ print "condition 14"
			return 4
		else:
			return 10
	#diagonale haut droite/bas gauche
	elif (thetaTortueBouge<=1.57 and thetaTortueBouge>=0) and (thetaTortueCompare<=4.71 and thetaTortueCompare>=3.14) or ((thetaTortueBouge<=4.71 and thetaTortueBouge>=3.14) and (thetaTortueCompare<=1.57 and thetaTortueCompare>=0)) :
		if yTortueBouge < yTortueCompare:
			#~ print "condition 15"
			return 3
		elif yTortueBouge > yTortueCompare: 
			#~ print "condition 16"
			return 3
		else:
			return 10
	else:
		#~ Condition inconnu on ecris les angles pour pouvoir la creer
		print "thetaTortueBouge = ",thetaTortueBouge,"thetaTortueCompare = ",thetaTortueCompare
		return 0

def noterPointDansMap(x,y,map,self):
	#On fait correspondre les coordonnees de la tortue avec la case correspondante du tableau en fonction de la precision choisie
	xBien=round(x,1)*ratio
	yBien=round(y,1)*ratio
	xBien=int(xBien)
	yBien=int(yBien)
	if (xBien <= nbrColonne and xBien >= debutColonne) and (yBien <= nbrLigne and yBien >= debutLigne):
		map[yBien][xBien]=1
		#~ print "la case rempli est x: ",xBien,"y: ",yBien,"// x=",x,"y=",y
	return map


def posTortueHautDroite(testCo,numTortue,self,tortueADroite,tortueEnHaut):
	if testCo == 1 and numTortue == 1:
		if self.x > self.x2:
			tortueADroite = 1
			#~ print "Tortue ",numTortue," est a droite"
		else:
			tortueADroite = 0
		if self.y > self.y2:
			tortueEnHaut = 1
		else:
			tortueEnHaut = 0
	elif testCo == 1 and numTortue == 2:
		if self.x2 > self.x:
			tortueADroite = 1
		else:
			tortueADroite = 0
		if self.y2 > self.y:
			tortueEnHaut = 1
		else:
			tortueEnHaut = 0
	elif testCo == 2 and numTortue == 1:
		if self.x > self.x3:
			tortueADroite = 1
		else:
			tortueADroite = 0
		if self.y > self.y3:
			tortueEnHaut = 1
		else:
			tortueEnHaut = 0
	elif testCo == 2 and numTortue == 3:
		if self.x3 > self.x:
			tortueADroite = 1
		else:
			tortueADroite = 0
		if self.y3 > self.y:
			tortueEnHaut = 1
		else:
			tortueEnHaut = 0
	elif testCo == 3 and numTortue == 2:
		if self.x2 > self.x3:
			tortueADroite = 1
		else:
			tortueADroite = 0
		if self.y2 > self.y3:
			tortueEnHaut = 1
		else:
			tortueEnHaut = 0
	elif testCo == 3 and numTortue == 3:
		if self.x3 > self.x2:
			tortueADroite = 1
		else:
			tortueADroite = 0
		if self.y3 > self.y2:
			tortueEnHaut = 1
		else:
			tortueEnHaut = 0
	return tortueADroite,tortueEnHaut

def chercherPointDansMap(x,y,theta,map,typeRecherche):
	xBien=round(x,1)*ratio
	yBien=round(y,1)*ratio
	xBien=int(xBien)
	yBien=int(yBien)
	compteurX=xBien
	compteurY=yBien
	ouvertureAngle=0
	cibleX = 0
	cibleY = 0
	trouver = 0
	coin=0
	if theta <0:
		theta +=6.28
	# Ici on va regarder le point non explorer le plus proche en fonction de l'orientation theta de la tortue cela permettant de faire avancer la tortue sans avoir des probleme de rond infini.
	if theta<=0.3925 or theta >5.8875:
		for compteurX in range(compteurX+1, nbrColonne+1,1):
			for compteurY in range(compteurY-ouvertureAngle,compteurY+ouvertureAngle+1,1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						#~ print x,y
						#~ print compteurX,compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurX % 2 == 0:
				ouvertureAngle += 1
	elif theta<=1.1775 and theta >0.3925:
		for compteurY in range(compteurY+1,nbrLigne+1,1):
			for compteurX in range(compteurX-ouvertureAngle,compteurX+ouvertureAngle+1,1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurY % 2 == 0:
				ouvertureAngle += 1
	elif theta<=1.9625 and theta >1.1775:
		for compteurY in range(compteurY+1,nbrLigne+1,1):
			for compteurX in range(compteurX-ouvertureAngle,compteurX+ouvertureAngle+1,1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurY % 2 == 0:
				ouvertureAngle += 1
	elif theta<=2.7475 and theta >1.9625:
		for compteurY in range(compteurY+1,nbrLigne+1,1):
			for compteurX in range(compteurX+ouvertureAngle,compteurX-ouvertureAngle-1,-1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurY % 2 == 0:
				ouvertureAngle += 1
	elif theta<=3.5325 and theta >2.7475:
		for compteurX in range(compteurX-1, debutLigne-1,-1):
			for compteurY in range(compteurY+ouvertureAngle,compteurY-ouvertureAngle-1,-1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurX % 2 == 0:
				ouvertureAngle += 1
	elif theta<=4.3175 and theta >3.5325:
		for compteurY in range(compteurY-1,debutLigne-1,-1):
			for compteurX in range(compteurX+ouvertureAngle,compteurX-ouvertureAngle-1,-1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurY % 2 == 0:
				ouvertureAngle += 1
	elif theta<=5.1025 and theta >4.3175:
		for compteurY in range(compteurY-1,debutLigne-1,-1):
			for compteurX in range(compteurX+ouvertureAngle,compteurX-ouvertureAngle-1,-1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurY % 2 == 0:
				ouvertureAngle += 1
	elif theta<=5.8875 and theta >5.1025:
		for compteurY in range(compteurY-1,debutLigne-1,-1):
			for compteurX in range(compteurX-ouvertureAngle,compteurX+ouvertureAngle+1,1):
				if (compteurY<=nbrLigne and compteurY>=debutLigne) and (compteurX<=nbrColonne and compteurX >= debutColonne):
					if map[compteurY][compteurX] == 0:
						cibleX=compteurX
						cibleY=compteurY
						trouver = 1
						break
			if trouver == 1:
				break
			if compteurY % 2 == 0:
				ouvertureAngle += 1
	if trouver == 0:
		#La on determine dans quel coin de la carte se trouve notre tortue pour pouvoir faire un balayage de toute la carte (parce que le balayage selon son orientation n'a rien trouve) qui soit un minimum intelligent.
		if x < 5.5:
			if y < 5.5:
				coin = 3
			else:
				coin = 4
		else:
			if y < 5.5:
				coin = 1
			else:
				coin = 2
		#On lance la grande recherche !
		(cibleX,cibleY)=grandeRechercheDansMap(map,coin)
		typeRecherche=2
	elif trouver == 1:
		typeRecherche=1
	else:
		print "erreur avec la recherche"
	#On envoi des cibles a atteindre au format attendu
	cibleX = float(cibleX) / float(ratio)
	cibleY = float(cibleY) / float(ratio)
	return cibleX,cibleY,typeRecherche

def grandeRechercheDansMap(map,coin):
	# Si la recherche par position et angle n'a pas fonctionne on lance un balayage de lacarte en fonction du coin ou ce trouve la tortue
	# On va donc balayer les 4 coins jusqua en trouver un ou il y a un point libre.
	# Le balayage se fait selon les points qui devraient se situer les plus proches.
	cibleX=0
	cibleY=0
	trouver=0
	if coin == 1:
		for y in range(moitierLigne+1,nbrLigne+1,1):
			for x in range(debutColonne,moitierColonne+1,1):
				if map[y][x] == 0:
					cibleX=x
					cibleY=y
					trouver = 1
					break
			if trouver == 1:
				break
		if trouver == 0:
			for y in range(moitierLigne+1,nbrLigne+1,1):
				for x in range(moitierColonne+1,nbrColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(moitierLigne,debutLigne-1,-1):
				for x in range(debutColonne,moitierColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(moitierLigne,debutLigne-1,-1):
				for x in range(moitierColonne+1,nbrColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
	elif coin == 2:
		for y in range(moitierLigne+1,nbrLigne+1,1):
			for x in range(moitierColonne+1,nbrColonne+1,1):
				if map[y][x] == 0:
					cibleX=x
					cibleY=y
					trouver = 1
					break
			if trouver == 1:
				break
		if trouver == 0:
			for y in range(moitierLigne+1,nbrLigne+1,1):
				for x in range(moitierColonne,0,-1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(moitierLigne,debutLigne-1,-1):
				for x in range(moitierColonne+1,nbrColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(moitierLigne,debutLigne-1,-1):
				for x in range(moitierColonne,-1,-1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
	elif coin == 3:
		for y in range(debutLigne,moitierLigne+1,1):
			for x in range(0,moitierColonne+1,1):
				if map[y][x] == 0:
					cibleX=x
					cibleY=y
					trouver = 1
					break
			if trouver == 1:
				break
		if trouver == 0:
			for y in range(debutLigne,moitierLigne+1,1):
				for x in range(moitierColonne+1,nbrColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(moitierLigne+1,nbrLigne+1,1):
				for x in range(debutColonne,moitierColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(moitierLigne+1,nbrLigne+1,1):
				for x in range(moitierColonne+1,nbrColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
	elif coin == 4:
		for y in range(debutLigne,moitierLigne+1,1):
			for x in range(moitierColonne+1,nbrColonne+1,1):
				if map[y][x] == 0:
					cibleX=x
					cibleY=y
					trouver = 1
					break
			if trouver == 1:
				break
		if trouver == 0:
			for y in range(moitierLigne+1,nbrLigne+1,1):
				for x in range(moitierColonne+1,nbrColonne+1,1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(debutLigne,moitierLigne+1,1):
				for x in range(moitierColonne,-1,-1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
		if trouver == 0:
			for y in range(moitierLigne+1,nbrLigne+1,1):
				for x in range(moitierColonne,-1,-1):
					if map[y][x] == 0:
						cibleX=x
						cibleY=y
						trouver = 1
						break
				if trouver == 1:
					break
	return cibleX,cibleY
	
def chercherPointNonAtteint(map):
	#Comme son nom l'indique cette fonction va nous dire si la tortue a explorer toute la carte ou non
	trouver = 0
	for y in range (debutLigne,nbrLigne+1,1):
		for x in range (debutColonne,nbrColonne+1,1):
			if map [y][x] == 0:
				trouver = 1
				break
		if trouver == 1:
			break
	return trouver
	
def posTortueHautDroiteA3(testCo,numTortue,self,tortueADroite,tortueEnHaut):
	#Quand on touche une autre tortue en reculant pour savoir ou la tortue est positionne afin de l'envoyer du bon coter pour eviter les colisions
	# 1 si la tortue est a droite ou en haut des deux autres, 0 si elle est a gauche ou en bas, 2 si elle est entre les deux
	if numTortue == 1:
		if self.x > self.x2 and self.x > self.x3:
			tortueADroite = 1
		if self.x < self.x2 and self.x < self.x3:
			tortueADroite = 0
		else :
			tortueADroite = 2
		if self.y > self.y2 and self.y > self.y3:
			tortueEnHaut = 1
		elif self.y < self.y2 and self.y < self.y3:
			tortueEnHaut = 0
		else:
			tortueEnHaut = 2
	elif numTortue == 2:
		if self.x2 > self.x and self.x2 > self.x3:
			tortueADroite = 1
		if self.x2 < self.x and self.x2 < self.x3:
			tortueADroite = 0
		else :
			tortueADroite = 2
		if self.y2 > self.y and self.y2 > self.y3:
			tortueEnHaut = 1
		elif self.y2 < self.y and self.y2 < self.y3:
			tortueEnHaut = 0
		else:
			tortueEnHaut = 2
	elif numTortue == 3:
		if self.x3 > self.x2 and self.x3 > self.x:
			tortueADroite = 1
		if self.x3 < self.x2 and self.x3 < self.x:
			tortueADroite = 0
		else :
			tortueADroite = 2
		if self.y3 > self.y2 and self.y3 > self.y:
			tortueEnHaut = 1
		elif self.y3 < self.y2 and self.y3 < self.y:
			tortueEnHaut = 0
		else:
			tortueEnHaut = 2
	return tortueADroite,tortueEnHaut

	
	


class TurtleCommander:
	def __init__(self,name,name2,name3):
		self.compteur = 0
		self.x=0
		self.y=0
		self.theta=0
		self.x2=0
		self.y2=0
		self.theta2=0
		self.x3=0
		self.y3=0
		self.theta3=0
		self.tortueVivante = 1
		self.tortueVivante2 = 1
		self.tortueVivante3 = 1
		
		rospy.wait_for_service("/kill")
		kill=rospy.ServiceProxy("/kill",Kill)
		tuer=kill("turtle1")
		
		rospy.wait_for_service("/spawn")
		spawn=rospy.ServiceProxy("/spawn",Spawn)
		
		#On fait un spawn des tortues aleatoire en fonction de la taille de la carte et du spawn des autres tortues
		self.x=random.randrange(debutMapX+1,tailleMapX-1,1)
		self.y=random.randrange(debutMapY+1,tailleMapY-1,1)
		nomDonne=spawn(self.x,self.y,0,name)
		print "created turtle " +nomDonne.name
		self.name=nomDonne.name
		
		self.x2=random.randrange(debutMapX+1,tailleMapX-1,1)
		self.y2=random.randrange(debutMapY+1,tailleMapY-1,1)
		while self.x == self.x2 and self.y == self.y2:
			self.x2=random.randrange(debutMapX+1,tailleMapX-1,1)
			self.y2=random.randrange(debutMapY+1,tailleMapY-1,1)
		nomDonne=spawn(self.x2,self.y2,0,name2)
		print "created turtle " +nomDonne.name
		self.name2=nomDonne.name
		
		self.x3=random.randrange(debutMapX+1,tailleMapX-1,1)
		self.y3=random.randrange(debutMapY+1,tailleMapY-1,1)
		while (self.x == self.x3 and self.y == self.y3) or (self.x2 == self.x3 and self.y2 == self.y3):
			self.x3=random.randrange(debutMapX+1,tailleMapX-1,1)
			self.y3=random.randrange(debutMapY+1,tailleMapY-1,1)
		nomDonne=spawn(self.x3,self.y3,0,name3)
		print "created turtle " +nomDonne.name
		self.name3=nomDonne.name
		
		self.pub = rospy.Publisher("/%s/cmd_vel"%self.name,Twist)
		rospy.Subscriber("/%s/pose"%self.name,Pose,self.pose_callback)
		
		self.pub2 = rospy.Publisher("/%s/cmd_vel"%self.name2,Twist)
		rospy.Subscriber("/%s/pose"%self.name2,Pose,self.pose_callback2)
		
		self.pub3 = rospy.Publisher("/%s/cmd_vel"%self.name3,Twist)
		rospy.Subscriber("/%s/pose"%self.name3,Pose,self.pose_callback3)
		
	def pose_callback(self,msg):
		if self.tortueVivante == 1:
			self.x=msg.x
			self.y=msg.y
			self.theta=msg.theta
		
	def pose_callback2(self,msg):
		if self.tortueVivante2 == 1:
			self.x2=msg.x
			self.y2=msg.y
			self.theta2=msg.theta
		
	def pose_callback3(self,msg):
		if self.tortueVivante3 == 1:
			self.x3=msg.x
			self.y3=msg.y
			self.theta3=msg.theta
			
	def command_loop(self):
		tourner=0
		tourner2=0
		tourner3=0
		dansMur=0
		dansMur2=0
		dansMur3=0
		avancer=1
		avancer2=1
		avancer3=1
		tempsMur=0
		tempsMur2=0
		tempsMur3=0
		droiteOuGauche=0
		droiteOuGauche2=0
		droiteOuGauche3=0
		repartir=1
		repartir2=1
		repartir3=1
		colision=0
		colision2=0
		colision3=0
		tempsColision=0
		tempsColision2=0
		tempsColision3=0
		angleCo=0
		angleCo2=0
		angleCo3=0
		tortueEnHaut = 0
		tortueEnHaut2 = 0
		tortueEnHaut3 = 0
		tortueADroite = 0
		tortueADroite2 = 0
		tortueADroite3 = 0
		typeRecherche = 0
		typeRecherche2 = 0
		typeRecherche3 = 0
		nvlCible=0
		nvlCible2=0
		nvlCible3=0
		map = [ [0] * (nbrColonne+1) for _ in range(nbrLigne+1)] #On cree une liste de liste pour representer notre map a taille O.1
		map2 = [ [0] * (nbrColonne+1) for _ in range(nbrLigne+1)] 
		map3 = [ [0] * (nbrColonne+1) for _ in range(nbrLigne+1)] 
		r=rospy.Rate(70)
		tempsAttente=time.time()
		#~ print self.x,self.y
		#On attend une seconde ici pour que l'on recoive les positions des tortues sinon ca nous renvoi 11-position au lieu de la position
		while time.time()-tempsAttente < 1:
			continue
		#~ print self.x,self.y
		cibleX = float(self.x)
		cibleY = float(self.y)
		cibleX2 = float(self.x2)
		cibleY2 = float(self.y2)
		cibleX3 = float(self.x3)
		cibleY3 = float(self.y3)
		parcourSansMap = time.time()
		continuer = 1
		while not rospy.is_shutdown() and continuer == 1:
			#On lance la fonction principale qui fait bouger les tortues
			if self.tortueVivante == 1 :
				(dansMur,tempsMur,droiteOuGauche,tourner,avancer,repartir,colision,tempsColision,angleCo,map,tortueEnHaut,tortueADroite,cibleX,cibleY,typeRecherche,nvlCible)=bougerTortue(self,self.x,self.y,self.theta,self.pub.publish,tourner,dansMur,tempsMur,avancer,droiteOuGauche,repartir,colision,tempsColision,angleCo,map,tortueEnHaut,tortueADroite,cibleX,cibleY,typeRecherche,nvlCible,parcourSansMap,1)
			if self.tortueVivante2 == 1 :
				(dansMur2,tempsMur2,droiteOuGauche2,tourner2,avancer2,repartir2,colision2,tempsColision2,angleCo2,map2,tortueEnHaut2,tortueADroite3,cibleX2,cibleY2,typeRecherche2,nvlCible2)=bougerTortue(self,self.x2,self.y2,self.theta2,self.pub2.publish,tourner2,dansMur2,tempsMur2,avancer2,droiteOuGauche2,repartir2,colision2,tempsColision2,angleCo2,map2,tortueEnHaut2,tortueADroite2,cibleX2,cibleY2,typeRecherche2,nvlCible2,parcourSansMap,2)
			if self.tortueVivante3 == 1 :
				(dansMur3,tempsMur3,droiteOuGauche3,tourner3,avancer3,repartir3,colision3,tempsColision3,angleCo3,map3,tortueEnHaut2,tortueADroite3,cibleX3,cibleY3,typeRecherche3,nvlCible3)=bougerTortue(self,self.x3,self.y3,self.theta3,self.pub3.publish,tourner3,dansMur3,tempsMur3,avancer3,droiteOuGauche3,repartir3,colision3,tempsColision3,angleCo3,map3,tortueEnHaut3,tortueADroite3,cibleX3,cibleY3,typeRecherche3,nvlCible3,parcourSansMap,3)
			if self.tortueVivante == 0 and self.tortueVivante2 == 0 and self.tortueVivante3 == 0:
				continuer = 0
			r.sleep()
	
rospy.init_node("exploration",anonymous=True)
commander=TurtleCommander("turtleA","turtleBob","turtleRobotLabo")
commander.command_loop()