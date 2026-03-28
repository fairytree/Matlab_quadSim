classdef MyStateSpace < nav.StateSpace & matlabshared.planning.internal.EnforceScalarHandle

    properties
        uniform_distro
        normal_distro
    end

    methods

        function obj = MyStateSpace(name, dim, state_min, state_max)
            obj@nav.StateSpace(name, dim, [state_min, state_max]);
            obj.normal_distro = matlabshared.tracking.internal.NormalDistribution(dim);
            obj.uniform_distro = matlabshared.tracking.internal.UniformDistribution(dim);
        end

        function bounded_state = enforceStateBounds(obj, state)
            bounded_state = min(max(state, obj.StateBounds(:,1)'), obj.StateBounds(:,2)');
        end

        function state = sampleUniform(obj, varargin)
            [sample_count, state_bounds] = obj.validateSampleUniformInput(varargin{:});
            obj.uniform_distro.RandomVariableLimits = state_bounds;
            state = obj.uniform_distro.sample(sample_count);
        end

        function state = sampleGaussian(obj, meanState, stdDev, varargin)
            assert(false);
        end

        function interp_state = interpolate(~, state_1, state_2, ratio)
            interp_state = state_1 + ratio' * (state_2 - state_1);
        end

        function dist = distance(~, state_1, state_2)
            dist = norm(state_2 - state_1);
        end

    end

end