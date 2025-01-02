function plotareayz(L, fig)
    % plotareayz - Plot dell'area di lavoro (workspace) di un manipolatore 3D nel piano Y-Z.
    %
    % Questa funzione calcola e visualizza l'area di lavoro di un manipolatore nel piano Y-Z.
    % La funzione utilizza la cinematica diretta per determinare la posizione del gripper
    % e rappresentare la proiezione del workspace rispetto alle variabili di giunto.
    %
    % Parametri:
    %   L   - Vettore contenente le lunghezze dei link del manipolatore [l1, l2, l3, l4, l5].
    %   fig - Numero della figura in cui visualizzare il plot.
    %
    % Note:
    %   - l'implementazione si riferisce in modo specifico al manipolatore assegnato.
    %
    % Esempio di utilizzo:
    %   L = [1, 1, 1, 1, 1];
    %   plotAreaYZ(L, 1);
    %
    % Vedi anche: plotareaxy

    l1i  = L(1);
    l2i  = L(2);
    l3ii = L(3);
    l1ii = l2i;
    l3i  = l1i/2;
    l4   = L(4);
    l5   = L(5);

    % Definire le equazioni di posizione dell gripper
    Y = @(theta2i, theta4, theta5) l3ii + l4 * cos(theta4) + l2i * sin(theta2i) + l5 * cos(theta4) * cos(theta5);
    Z = @(theta2i, theta4, theta5) -l4 * sin(theta4) - l5 * cos(theta5) * sin(theta4);

    % Intervalli degli angoli di giunto
    theta2samples = linspace(0, pi, 30);
    theta4samples = linspace(0, 2 * pi, 30);

    % Preparazione della figura
    figure(fig)
    hold on
    title("Workspace del manipolatore (Y-Z)")
    xlabel("Y")
    ylabel("Z")
    axis equal
    % Inizializzazione array di punti
    y = [];
    z = [];
    % Creazione dell'area di lavoro
    for theta2i = theta2samples
        for theta4 = theta4samples
            % Calcolo delle coordinate del punto finale
            y = [y Y(theta2i, theta4, 0)];
            z = [z Z(theta2i, theta4, 0)];
        end
    end

    plot(y, z, 'g-',LineWidth=2)

    hold off
end


