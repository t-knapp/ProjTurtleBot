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
		|	   ENEMY-GOAL			* Bällpositionen
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

- Robot Orientierung, zu Bande Gegner-Tor, Eigenes-Tor gerichtet.
	- Grad zum Gegner Tor
- Roboter Position x,y
- Ballposition x,y
- Distanz (optional)
	- Geschwindigkeit anpassen?
	 
		