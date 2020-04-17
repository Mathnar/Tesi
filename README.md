# Tesi 
10/4/2020

Load.py: Contiene il programma totale. Crea matrice adiacenza e grafo dati i satelliti IRIDIUM (32 satelliti)

MathTest.py: Contiene l'algorimto (diviso in 3 fuznioni) che ho scritto per trovare quando un satellite è visibile da un altro.
 - "vectp" trova vettore parallelo
 - "rectEq" trova l'equzione della retta passante per i due punti dati in coordinate xyz e risolve il sistema 
 formato dalle equazioni della retta e l'equazione della sfera (rappresentante la terra approssimata a sfera perfetta).
 - "isVisible" sfrutta il precedente algorimto per trovare se la soluzione restituita dal calcolatore è reale o complessa. 
 Se reale, la retta interseca la sfera. Ne segue che i due satelliti non "si vedono". 
 Se complessa, la retta non interseca la sfera. Ne segue che i due satelliti "si vedono".
 Nel programma totale (load) queste 3 funzioni sono ridotte ad un'unica funzione.
 
 Next.py: Script dove eseguo tutti gli stessi test ma utilizzando la costellazione IRIDIUM NEXT (75 satelliti)
 
 DtnS.py: Script contenente il test del simulatore pydtnsim 
 
 Dijkstra.py: Script contenente alcune funzioni (che per ora non ho utilizzato) per il calcolo del percorso ottimale di Dijkstra e creazione di grafo e 
 matrice delle adiacenze
 
 Tutti gli altri file sono script provvisori su cui mi sono appuntato delle funzioni o dei test.
 
