function dec = bit2dec(bits)
%BIT2DEC Transform an array of bits (e.g. 0 and 1s) to a decimal
     str = num2str(bits);
     str(isspace(str)) = '';

     dec = bin2dec(str);
end

