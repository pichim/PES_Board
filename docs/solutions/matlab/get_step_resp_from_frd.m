function step_resp = get_step_resp_from_frd(G, f_max)

    % Extract complex frequency response
    g = squeeze(G.ResponseData);
    
    % Reconstruct artificial DC (simulate symmetry at zero freq)
    g_dc = g(1); % Use g[0] again as a placeholder for DC
    g = [g_dc; g]; % Prepend DC

    % Extend frequency vector accordingly
    freq = [0; G.Frequency(:)];

    % Zero out above f_max_hz
    g(freq > f_max) = 0;

    % Construct full symmetric spectrum
    g_full = [g; conj(g(end-1:-1:2))];

    % Step response is cumulative sum of real part of IFFT
    step_resp = cumsum(real(ifft(g_full)));

end
