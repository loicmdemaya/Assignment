Dinamica del Veicolo – Compito 1
Confronto tra Modelli: Cinematico, Lineare, Non Lineare

Studente: [STEVE LOIC SAFACK MDEMAYA]
Corso: PAAD – Pianificazione e Controllo
Argomento: Simulazione e confronto di modelli di dinamica del veicolo
Obiettivo

L’obiettivo di questo progetto è implementare e confrontare tre modelli a bicicletta del veicolo in diversi scenari di guida:

    Modello cinematico

    Modello lineare

    Modello non lineare (con pneumatici secondo il modello di Pacejka)

Le simulazioni servono a valutare l’accuratezza dei modelli in relazione a differenti velocità e input di sterzata.
 Descrizione dei Modelli
1. Modello Cinematico

    Assume l’assenza di slittamento laterale

    Ideale per manovre a bassa velocità

    Non tiene conto della dinamica degli pneumatici

2. Modello Lineare

    Approssimazioni per piccoli angoli

    Le forze degli pneumatici sono funzioni lineari degli angoli di deriva

    Adatto a velocità moderate

3. Modello Non Lineare (Pacejka)

    Utilizza la Magic Formula per calcolare le forze degli pneumatici

    Tiene conto della saturazione delle forze e di grandi angoli di slittamento

    Ideale per alte velocità e manovre aggressive

 Simulazioni e Osservazioni
Esercizio 1: Sterzata Sinusoidale

    Input: δ(t) = 0.1·sin(2π·0.5·t) rad

    Velocità: 10 m/s e 27 m/s

    Accelerazione: 0.0 m/s²

Osservazioni:

    A 10 m/s: Tutti i modelli si comportano in modo simile

    A 27 m/s:

        Cinematico: Traiettoria troppo ottimista

        Lineare: Inizia a derapare

        Non lineare: Mostra sottosterzo e saturazione delle forze degli pneumatici

Esercizio 2: Sterzata Costante con Accelerazione

    Caso 1: δ = 0.01 rad, ax = 1.0 m/s², vx = 24 m/s

    Caso 2: δ = 0.055 rad, ax = 1.0 m/s², vx = 24 m/s

Osservazioni:

    Caso 1: Tutti i modelli producono risultati accettabili

    Caso 2:

        Cinematico: Sottostima il movimento laterale

        Lineare: Inizia a fallire a causa della non linearità

        Non lineare: Rappresenta accuratamente saturazione e deviazione dalla traiettoria

Esercizio 3: Sterzata Improvvisa (Step Input)

    Input: δ(t) = 0 fino a t = 2 s, poi δ = 0.05 rad

    Velocità: 24 m/s

    Accelerazione: 0.0 m/s²

Osservazioni:

    Cinematico: Risposta immediata, ma irrealistica

    Lineare: Ritardo leggero, realismo limitato

    Non lineare: Risposta progressiva, con sottosterzo realistico

