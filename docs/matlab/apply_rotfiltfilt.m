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
function xf = apply_rotfiltfilt(G, sinarg, x)

    % signal size
    [Nx, nx] = size(x);
    xf = zeros(Nx, nx);
    p = exp(1j*(sinarg));
    
    for i = 1:nx
        % eliminate mean
        y = x(:,i) - mean(x(:,i));
        yR = y .* p;
        yQ = y .* conj(p);
        % filtering in transformed coordinates
        yR = filtfilt(G.num{1}, G.den{1}, yR);
        yQ = filtfilt(G.num{1}, G.den{1}, yQ);
        % back transformation
        % xf(:,i) = real((yR.*conj(p) + yQ.*p)*0.5); % scaling does not
        % matter if we build the raio anyways
        xf(:,i) = real((yR.*conj(p) + yQ.*p));
    end

end