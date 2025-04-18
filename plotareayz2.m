function plotareayz2(L, fig)
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
    nsamples = 30;
    theta2samples = [zeros(1,nsamples), linspace(0, pi/2, nsamples), ones(1,nsamples)*pi/2];
    theta4samples = [linspace(pi/2, 3/2*pi, 30), ones(1,nsamples)*3/2*pi, linspace(-pi/2, pi/2, 30)];

    % Preparazione della figura
    figure(fig)
    hold on
    title("Workspace del manipolatore (Y-Z)")
    xlabel("Y")
    ylabel("Z")
    axis equal
    % Inizializzazione array di punti
    z = [];
    y = [];
    % Creazione dell'area di lavoro
    for idx = 1:1:nsamples*3
        theta2i = theta2samples(idx);
        theta4 = theta4samples(idx);
        % Calcolo delle coordinate del punto finale
        y = [y Y(theta2i, theta4, 0)];
        z = [z Z(theta2i, theta4, 0)];
    end
    plot([y, y(1)], [z, z(1)], 'g-',LineWidth=2)
    plotmanipulatoryz([pi/2, 0, 0], L, 'k', fig)
    legend ('Workspace','Links','','','Joints','CW')
    hold off
end


