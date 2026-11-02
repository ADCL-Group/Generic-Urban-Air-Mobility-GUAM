classdef ref_model_mask

    methods(Static)

        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle 
        %  - MaskObject 
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.
        function MaskInitialization(maskInitContext)
            maskWorkspace = maskInitContext.MaskWorkspace;

            r_omega = maskWorkspace.get('r_omega');
            p_omega = maskWorkspace.get('p_omega');
            y_omega = maskWorkspace.get('y_omega');
            h_omega = maskWorkspace.get('h_omega');

            r_damp = maskWorkspace.get('r_omega');
            p_damp = maskWorkspace.get('p_damp');
            y_damp = maskWorkspace.get('y_damp');
            h_damp = maskWorkspace.get('h_damp');

            % omega = [r_omega; p_omega; y_omega; h_omega];
            % damp = [r_damp; p_damp; y_damp; h_damp];

            omega = [r_omega; p_omega; h_omega; h_omega];
            damp = [r_damp; p_damp; h_damp; h_damp];

            maskWorkspace.set('omega',omega);
            maskWorkspace.set('damp',damp);
            maskWorkspace.set('y_omega',y_omega);
            maskWorkspace.set('y_damp',y_damp);
        end

        % Use the code browser on the left to add the callbacks.

    end
end