# Tesi 
10/4/2020

Iridium: Crea per un tempo stabilito, ad intervalli di tempo stabiliti un file di testo "prova.txt" contenente tutte le matrici delle adiacenze, di seguito ad un indice.

EarthSat: Trova, date le coordinate di un punto sulla terra, il satellite ad esso più vicino. Lo utilizzo per trovare il satellite sorgente e quello destinazione.

Cgr: Per prima cosa esegue lo script EarthSat ed acquisisce il nodo sorgente e destinazione sulla rete. Successivamente esegue la routine di Contact Graph Routing utilizzando funzioni appositamente scritte. Il risultato è dato paragonandolo con l'esecuzione di un banale algoritmo di Dijkstra.
-Funzioni:
"load_CP" funzione che dato un indice e la variabile booleana update, legge dal file di testo file.txt la corretta matrice di adiacenza per l'attuale istante di tempo e restituisce un Contact Graph ("c_graph") ed una matrice di adiacenza contenente solo i vertici di adiacenza ("mtx").
"getInd": ritorna l'indice corretto in base al presente istante di tempo.
"getRl": funzione che costruisce passo passo la route list dal nodo sorgente alla destinazione. Prende in ingresso il Contact Graph attuale, l'indice del nodo attuale (che cambiarà via via che verrà agginunto alla route list), l'indice del nodo destinazione e la lista contenente i nodi "esclusi".


MathTest.py: Contiene l'algorimto (diviso in 3 fuznioni) che ho scritto per trovare quando un satellite è visibile da un altro.
 - "vectp" trova vettore parallelo
 - "rectEq" trova l'equzione della retta passante per i due punti dati in coordinate xyz e risolve il sistema 
 formato dalle equazioni della retta e l'equazione della sfera (rappresentante la terra approssimata a sfera perfetta).
 - "isVisible" sfrutta il precedente algorimto per trovare se la soluzione restituita dal calcolatore è reale o complessa. 
 Se reale, la retta interseca la sfera. Ne segue che i due satelliti non "si vedono". 
 Se complessa, la retta non interseca la sfera. Ne segue che i due satelliti "si vedono".
 Nel programma totale (load) queste 3 funzioni sono ridotte ad un'unica funzione.

 
 Tutti gli altri file sono script provvisori su cui mi sono appuntato delle funzioni o dei test.
 
