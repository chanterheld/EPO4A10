function [ind] = findFpeaks(vec, threshold)
bin_vec = (vec >= threshold);

for ind = 1:length(bin_vec)
    if bin_vec(ind)        
        break;
    end
end
end

