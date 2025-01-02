function [phi] = invdyn(Mabs, Habs, Labs, I, m, G, Phie)
    % Funzione per la dinamica inversa di un robot a catena cinematica
    % aperta
    %
    % INPUT:
    % Mabs - Matrice di trasformazione del sistema di riferimento assoluto per ogni link
    % Wabs - Velocità angolare assoluta di ogni link
    % Habs - Forze e momenti assoluti per ogni link
    % Labs - Posizione del giunto successivo per ogni link
    % I - Tensore di inerzia per ogni link rispetto al baricentro
    % m - Massa di ogni link
    % G - Posizione del baricentro di ogni link rispetto al sistema locale del link
    % Phie - Forze esterne applicate sul gripper
    %
    % OUTPUT:
    % phi - Forze e momenti per ogni giunto

    % Inizializzazione
    numLinks = size(I, 3);
    Phi(:,:,numLinks) = zeros(4,4); % Forze e momenti iniziali al gripper
    Hg = [0, 0, 0, -9.81;  % Forza gravitazionale applicata
          0, 0, 0, 0;
          0, 0, 0, 0;
          0, 0, 0, 0];
    
    % Forze esterne applicate al gripper
    Phistar(:,:,numLinks) = Phie;
    phi(:,:,numLinks) = zeros(4,4);

    % Ricorsione inversa per calcolare le azioni sui giunti
    for j = numLinks-1:-1:1  % Iteriamo dal penultimo link verso il primo
        % Calcolo degli pseudo-tensori di inerzia per ciascun link
        pseudoJ = pseudoinertia(I(:,:,j), m(j), G(:,:,j));
        
        % Trasformazione degli pseudo-tensori di inerzia al sistema di riferimento assoluto
        pseudoJ = Mabs(:,:,j) * pseudoJ * Mabs(:,:,j)';
        
        % Calcolo delle forze di inerzia e del peso del link
        Phi(:,:,j) = -((Habs(:,:,j) - Hg) * pseudoJ - ((Habs(:,:,j) - Hg) * pseudoJ)');
        
        % Calcolo le azioni applicate al giunto j
        Phistar(:,:,j) = Phi(:,:,j) + Phistar(:,:,j+1);
        
        % Calcolo della forza risultante sul giunto j (con funzione pseudot)
        phi(:,:,j) = pseudot(-Phistar(:,:,j), Labs(:,:,j));
    end
end
