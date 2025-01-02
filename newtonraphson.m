function [Qi, i] = newtonraphson(F, J, Q0, S, tol, imax)
    % NEWTONRAPHSON Calcola uno zero di un sistema di equazioni non lineari
    % utilizzando l'algoritmo di Newton-Raphson.
    %
    % Sintassi:
    %   [Qi, i] = newtonraphson(F, J, Q0, S, tol, imax)
    %
    % Input:
    %   F       - Funzione anonima che rappresenta il sistema di equazioni
    %             non lineari. Deve restituire un vettore di dimensione (n,1).
    %
    %   J       - Funzione anonima che rappresenta il Jacobiano del sistema.
    %
    %   Q0      - Vettore iniziale (approssimazione iniziale) per l'algoritmo.
    %
    %   S       - Vettore obiettivo per il test di convergenza.
    %
    %   tol     - Tolleranza per il criterio di convergenza.
    %
    %   imax    - Numero massimo di iterazioni.
    %
    % Output:
    %   Qi      - Vettore soluzione approssimata che rende F(Qi) vicino a S.
    %   i       - Numero di iterazioni usate per trovare la soluzione.
    
    % Inizializzazione
    Qi = Q0;
    Si = F(Qi);
    i = 0;

    % Iterazione di Newton-Raphson
    while i < imax
        % Calcola l'aggiornamento di Qi usando il Jacobiano
        JQi = J(Qi);

        % Verifica se il Jacobiano è invertibile
        if abs(det(JQi)) < 1e-12
            error('Il Jacobiano è prossimo alla singolarità alla iterazione %d. Impossibile continuare.', i);
        end

        % Calcola Qi+1 e Si+1
        Qi = Qi + JQi\ (S - Si);
        Si = F(Qi);

        % Test di convergenza
        if norm(S - Si, inf) < tol
            fprintf('Soluzione trovata in %d iterazioni.\n', i);
            return;
        end

        % Incrementa il contatore di iterazioni
        i = i + 1;
    end

    % Avviso se il numero massimo di iterazioni è raggiunto senza convergenza
    warning('Numero massimo di iterazioni raggiunto senza trovare la soluzione.');
end
