�}`I  �	  [�������e��'�5X,��c˶�/��$��MCg��O'���MiE���u3Ec
��Y�.c��#
�����j5��Ėy��D���nh/�o�1*�G��,��O�h^L�	H�u���WlW6ﶘ�p��!��d{e6�+w'M�M�U�f>�l�M�F^�0Z`�BgD���"%/�F��]�w�/��v�	fN�ؔ��6�����_`c�2|U#څ����8qB*�_)f�R�8�쯲ث�Q����ʢټ�����6�:9ґ�͘V�K
�'�
��{���2�|���Q�"�o��ܩ�̣��Pp$�0�'i���!g����������J��x��o�Ok����'�[��2�E�C��4�=��Ρnx��q���� 9#�y+�(�j�\������,%����oh�N�_�B�:�yX]t�+�|q���T�5�n4��ꢗ5�-rn���<j��CZ��_Վt���%�32�j��A�:L�\�b{�'{\ach point on path
           
       robot.animate(tg(p,:), handles);
       drawnow            
                 
    end

         

    % save the last joint angles away in the graphical robot
    for handle=handles'
        h = get(handle, 'UserData');
        h.q = tg(end,:);
        set(handle, 'UserData', h);
    end
end


