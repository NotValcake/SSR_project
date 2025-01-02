function plotareaxy(L, fig)
    % PLOTAREAXY - Plot dell'area di lavoro (workspace) di un manipolatore 3D nel piano X-Y.
    %
    % Questa funzione calcola e visualizza l'area di lavoro di un manipolatore nel piano X-Y.
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
    %   plotareaxy(L, 1);
    %
    % Vedi anche: plotareayz

    l1i  = L(1);
    l2i  = L(2);
    l3ii = L(3);
    l1ii = l2i;
    l3i  = l1i/2;
    l4   = L(4);
    l5   = L(5);

    % Definire le equazioni di posizione del gripper
    X = @(theta2i, theta5) l1i - l3i + l2i * cos(theta2i) - l5 * sin(theta5);
    Y = @(theta2i, theta4, theta5) l3ii + l4 * cos(theta4) + l2i * sin(theta2i) + l5 * cos(theta4) * cos(theta5);
    
    % Intervalli degli angoli di giunto
    theta2samples = linspace(0, pi, 30);
    theta5samples = linspace(0, 2 * pi, 30);

    % Preparazione della figura
    figure(fig)
    hold on
    title("Workspace del manipolatore (X-Y)")
    xlabel("X")
    ylabel("Y")
    axis equal
    % Inizializzazione array di punti
    x = [];
    y = [];
    % Creazione dell'area di lavoro
    for theta2i = theta2samples
        for theta5 = theta5samples
            % Calcolo delle coordinate del punto finale
            x = [x X(theta2i, theta5)];
            y = [y Y(theta2i, 0, theta5)];
        end
    end

    plot(x, y, 'g-', LineWidth=2)

    hold off
end


