function y = filterTeensy( port, signal )
% MATLAB <-> Teensy data transfer over serial bus
%
% 'port' is a serial object, e.g.: port = serial('COM11');
% 'signal' is a vector representing the input signal
%
% Javier Lombillo
% 2017-02-15

tic;
blocksize = 512; % number of samples in each transfer

n = length( signal );
numblocks = ceil( n / blocksize );

fprintf( 'Sending %d blocks of data to be filtered...\n', numblocks )

sig_idx = 1;
out_idx = 1;

for i = 1 : numblocks
    for j = 1 : blocksize

        % out of data, send EOF character (+Inf)
        if sig_idx == (n + 1)
            fwrite( port, Inf, 'float' );
            break;
        end
        
        % otherwise, send sample data as raw binary bytes
        fwrite( port, signal(sig_idx), 'float' );
        sig_idx = sig_idx + 1;
    end
    
    % wait for Teensy to start sending data back
    while port.BytesAvailable == 0
        ; % do nothing
    end
    
    for j = 1 : blocksize

        % last sample, so bail out
        if out_idx == (n + 1)
            break;
        end
        
        y(out_idx) = fread( port, 1, 'float' );
        out_idx = out_idx + 1;
    end
end 
toc
end