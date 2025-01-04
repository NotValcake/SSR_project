function plotareaxz2(L, fig)
    % PLOTAREAXZ2 - Plot dell'area di lavoro (workspace) di un manipolatore 3D nel piano X-Z.
    %
    % Questa funzione calcola e visualizza l'area di lavoro di un manipolatore nel piano X-Z.
    % La funzione utilizza la cinematica diretta per determinare la posizione del gripper
    % e rappresentare la proiezione del workspace rispetto alle variabili di giunto.
    %
    % Parametri:
    %   L   - Vettore contenente le lunghezze dei link del manipolatore [l1, l2, l3, l4, l5].
    %   fig - Numero della figura in cui visualizzare il plot.
    %
    % Note:
    %   - l'implementazione si riferisce in modo specifico al manipolatore assegnato.
    %   - la rotazione theta2 del giunto J2 si ritiene limitata tra 0 e pi.
    %
    % Esempio di utilizzo:
    %   L = [1, 1, 1, 1, 1];
    %   plotareaxz2(L, 1);
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
    Z = @(theta2i, theta4, theta5) -l4 * sin(theta4) - l5 * cos(theta5) * sin(theta4);

    % Intervalli degli angoli di giunto
    nsamples = 15;
    theta2samples = [zeros(1, nsamples), zeros(1, nsamples), linspace(0, pi, nsamples), ones(1, nsamples)*pi,...
        ones(1, nsamples)*pi, ones(1, nsamples)*pi, linspace(pi, 2*pi, nsamples), ones(1, nsamples)*2*pi];
    theta4samples = [linspace(-pi/2, pi/2, nsamples), ones(1, nsamples)*pi/2, ones(1, nsamples)*pi/2, ones(1, nsamples)*pi/2,...
        linspace(pi/2, 3*pi/2, nsamples), ones(1, nsamples)*3*pi/2, ones(1, nsamples)*3*pi/2, ones(1, nsamples)*3*pi/2];
    theta5samples = [ones(1, nsamples)*-pi/2, linspace(-pi/2, 0, nsamples), zeros(1, nsamples), linspace(0, pi/2, nsamples),...
        ones(1, nsamples)*pi/2, linspace(pi/2, 0, nsamples), zeros(1,nsamples), linspace(0, -pi/2, nsamples)];

    % Preparazione della figura
    figure(fig)
    hold on
    title("Workspace del manipolatore (X-Z)")
    xlabel("X")
    ylabel("Z")
    axis equal
    
    % Inizializzazione array di punti
    x = [];
    y = [];
    z = [];

    % Creazione dell'area di lavoro
    for idx = 1:1:nsamples*8
        theta2i = theta2samples(idx);
        theta4 = theta4samples(idx);
        theta5 = theta5samples(idx);
        
        % Calcolo delle coordinate del punto finale
        x = [x X(theta2i, theta5);];
        y = [y Y(theta2i, theta4, theta5);];
        z = [z Z(theta2i, theta4, theta5);];
    end
    
    plot(x, z, 'g-', LineWidth=2)


    plotmanipulatorxz([pi/2, 0, 0], L, 'k', fig)
    legend ('Workspace','Links','','','Joints','CW')
    hold off
end


