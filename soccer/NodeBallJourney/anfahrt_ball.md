# Anfahrt Ball

## Fälle
- Ball liegt in einer der 4 Ecken
	- Fallunterscheidung je nach ecke
- Ball liegt an einer Bande links oder Rechts
	- Fallunterscheidung jeh seite
- Ball liegt an einer Kopfbande
	- Fallunterscheidung jeh Tor
- Ball liegt im Feld
- _Steht Roboter __vor__ oder __hinter__ dem Ball_

		^y
		|	   ENEMY-GOAL			* Bällpositionen / Fälle
		|	_____:::::_____			
		|	|*         * *|
		|	|             |
		|	|     (o)     |
		|	|             |
		|	|*     *     *| 
		|	|             |
		|	|     (o)     |
		|	|             |
		|	|* *         *|
		|	-----:::::-----		
		|							x
		|------------------------->
		
		
## Benötigte Daten

- Ballposition x,y
	- 	zur Fallentscheidung, wie liegt der Ball
- Roboter Position x,y
- 	- zur Fallentscheidung wie fhare ich zum Ball
- Robot Orientierung, zu Bande Gegner-Tor, Eigenes-Tor gerichtet.
	- Grad zum Gegner Tor
	- zur Fallentscheidung wie fhare ich zum Ball
- Distanz zum Ball (optional)
	- Geschwindigkeit anpassen?

## Vorliegende Daten
- Bmall liegt links oder Rechts vom Roboter (im sichtfeld)
- Distanz zum Ball

		
## 