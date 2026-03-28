classdef MyStateValidator < nav.StateValidator & matlabshared.planning.internal.EnforceScalarHandle

    properties
        rectangular_obs;
    end

    methods

        function obj = MyStateValidator(space, params)
            obj@nav.StateValidator(space);
            obj.rectangular_obs = params.rectangular_obs;
        end

        % c) Define how a given state is validated. The STATE input can be a
        %    single state (row) or multiple states (one state per row).
        %    You have to customize this function if you want to have
        %    special validation behavior, for example to check for
        %    collisions with obstacles.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isStateValid
        %
        function isValid = isStateValid(obj, state)

            narginchk(2,2);

            nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                "isStateValid", "state");

            % Default behavior: Verify that state is within bounds
            bounds = obj.StateSpace.StateBounds';
            inBounds = state >= bounds(1,:) & state <= bounds(2,:);
            isValid = all(inBounds, 2);

        end

        % d) Define how a motion between states is validated.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isMotionValid
        %
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)

            narginchk(3,3);

            state1 = nav.internal.validation.validateStateVector(state1, ...
                obj.StateSpace.NumStateVariables, "isMotionValid", "state1");
            state2 = nav.internal.validation.validateStateVector(state2, ...
                obj.StateSpace.NumStateVariables, "isMotionValid", "state2");

            if (~obj.isStateValid(state1))
                error("statevalidator:StartStateInvalid", "The start state of the motion is invalid.");
            end

            % Default behavior: Interpolate with some fixed interval
            % between state1 and state2 and validate the interpolated
            % points.
            numInterpPoints = 100;
            interpStates = obj.StateSpace.interpolate(state1, state2, linspace(0,1,numInterpPoints));
            interpValid = obj.isStateValid(interpStates);
            firstInvalidIdx = find(~interpValid, 1);

            if isempty(firstInvalidIdx)
                isValid = true;
                lastValid = state2;
            else
                isValid = false;
                lastValid = interpStates(firstInvalidIdx-1,:);
            end

            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end

    end
end