function p = pseudot(a, b)
    % Calcola lo pseudo prodotto scalare tra due matrici 4x4, a e b
    % INPUT:
    % a, b - Matrici 4x4
    %
    % OUTPUT:
    % p - Risultato scalare dello pseudo dot product

    % Estrai i termini specifici di a e b
    p = a(1,4) * b(1,4) + ... % a14 * b14
        a(2,4) * b(2,4) + ... % a24 * b24
        a(3,4) * b(3,4) + ... % a34 * b34
        a(3,2) * b(3,2) + ... % a32 * b32
        a(1,3) * b(1,3) + ... % a13 * b13
        a(2,1) * b(2,1);      % a21 * b21
end
