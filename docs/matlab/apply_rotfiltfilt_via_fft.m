%
% This file is part of pichim's controller tuning framework.
%
% This sofware is free. You can redistribute this software
% and/or modify this software under the terms of the GNU General
% Public License as published by the Free Software Foundation,
% either version 3 of the License, or (at your option) any later
% version.
%
% This software is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%
% See the GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public
% License along with this software.
%
% If not, see <http:%www.gnu.org/licenses/>.
%
%%
function xf = apply_rotfiltfilt_via_fft(G, sinarg, x, Ts)

    % signal size
    [Nx, nx] = size(x);
    xf = zeros(Nx, nx);
    p = exp(1j*(sinarg));

    df = 1 / (Nx * Ts);
    fnyq = 1 / (2 * Ts);
    freq = (0:df:2*fnyq-df).';
    g = squeeze(freqresp(G, 2*pi*freq));
    g = g .* conj(g);
    
    for i = 1:nx
        % eliminate mean
        y = x(:,i) - mean(x(:,i));
        yR = y .* p;
        yQ = y .* conj(p);
        % filtering in transformed coordinates
        yR = filtfilt_fft(g, yR);
        yQ = filtfilt_fft(g, yQ);
        % back transformation
        % xf(:,i) = real((yR.*conj(p) + yQ.*p)*0.5); % scaling does not
        % matter if we build the raio anyways
        xf(:,i) = real((yR.*conj(p) + yQ.*p));
    end

end

function y = filtfilt_fft(g, x)

    y = ifft( fft(x) .* g );

end

