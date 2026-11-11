
MODULE EXAMPLE
    ! module data
    PERS tooldata gun := [TRUE, [[0.000, 0.000, 200.000],[1.000000000, 0.000000000, 0.000000000, 0.000000000]], [2.5, [0.000, 0.000, 100.000], [1, 0, 0, 0], 0, 0, 0]];
    PERS wobjdata table := [FALSE, TRUE, "", [[1000.000, 0.000, 400.000],[1.000000000, 0.000000000, 0.000000000, 0.000000000]], [[0.000, 0.000, 0.000],[1.000000000, 0.000000000, 0.000000000, 0.000000000]]];
    CONST speeddata v_weld := [50.000, 100.0000, 1000.000, 100.0000];
    CONST zonedata z_weld := [FALSE, 1.000, 1.500, 1.500, 0.1500, 1.500, 0.1500];
    CONST robtarget p_home := [[600.000, -100.000, 800.000],[0.707106781, 0.000000000, 0.707106781, 0.000000000],[0, 0, 0, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST jointtarget j_park := [[0.0000, 0.0000, 0.0000, 0.0000, 30.0000, 0.0000], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST pos offset := [0.000, 0.000, 100.000];
    PROC main()
        VAR bool done := FALSE;
        ConfL \On;
        MoveAbsJ j_park, v_weld, z_weld, gun;
        MoveJ p_home, v_weld, z_weld, gun \WObj:=table;
        MoveL [[600.000, -100.000, 800.000],[0.707106781, 0.000000000, 0.707106781, 0.000000000],[-1, 0, 1, 1],[250.000,90.0000,9E+09,9E+09,9E+09,9E+09]], v_weld, z_weld, gun \WObj:=table;
        MoveC [[600.000, -100.000, 800.000],[0.707106781, 0.000000000, 0.707106781, 0.000000000],[0, 0, 0, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], p_home, v_weld, z_weld, gun \WObj:=table;
        WHILE NOT done DO
            FOR i FROM 1 TO 3 DO
                SetDO do_torch, 1;
                WaitTime 0.5;
                SetAO ao_voltage, 24.5;
            ENDFOR
            WaitDI di_part, 1;
            done := TRUE;
        ENDWHILE
        TPWrite "done: ""pass"" \\ ok";
        Stop;
    ENDPROC
ENDMODULE
