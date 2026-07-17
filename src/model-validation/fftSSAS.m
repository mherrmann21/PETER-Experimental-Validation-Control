function [P1,f] = fftSSAS(X, Fs)
    %% Compute the single-sided amplitude spectrum of a signal via fft
    % Similar to the MATLAB docs for fft.
    arguments (Input)
        % Signal as a matrix or vector, where each column is treated as a
        % separate signal
        X   (:,:) double

        % Sampling frequency
        Fs  (1,1) double
    end

    % Get data with an even number of sampling points
    LComplete = size(X,1);
    X = X(1:end-rem(LComplete,2),:);

    % Get signal length after cropping the data
    L = size(X,1);

    % Two-sided amplitude spectrum
    P2Gyr = abs(fft(X)/L);

    % Single-sided spectrum
    P1 = P2Gyr(1:L/2+1,:);
    P1(2:end-1) = 2*P1(2:end-1);

    % Frequency vector for fft output
    f = Fs/L*(0:(L/2));
end