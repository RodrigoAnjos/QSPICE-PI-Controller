## Copyright (C) 2024 Rodrigo
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

## -*- texinfo -*-
## @deftypefn {} {@var{retval} =} CompareArrays (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Rodrigo <Rodrigo@DESKTOP-A2SPLM8>
## Created: 2024-04-03
addpath(pwd);
%%function DifY = CompareArrays(RefTimeArray, RefYArray, OpTimeArray, OpYArray)
function DifY = CompareArrays(x)
  % Find the corresponding values in the OpTimeArray that correspond to the RefTimeArray
  %for t = RefTimeArray
    % While the vector is not empty keep shrinking the epsilon
  %  epsilon = RefTimeArray(1)-RefTimeArray(0);
  %  printf("%d\n",epsilon);
    %do
    %  prev_loc = loc;
    %  loc = find(RefTimeArray < t+epsilon & OpTimeArray > t-epsilon);
    %  epsilon = epsilon *0.9;
    %until !isempty(loc)
  %endfor;
  DifY = x;
endfunction
