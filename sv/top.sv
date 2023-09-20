interface adc_if (input clk, rstn);
    logic [11:0] adc_out;
endinterface

package pkg_lib;

    import uvm_pkg::*;
    `include "uvm_macros.svh"

    typedef bit [11:0] adc_resolution;

    `define PI 3.141592654
    `define FREQ 2e6
    `define TSTEP 1e-9
    `define NBITS 12

    class adc_item extends uvm_sequence_item;

        rand adc_resolution data;

        `uvm_object_utils_begin(adc_item)
            `uvm_field_int(data, UVM_DEFAULT)
        `uvm_object_utils_end

        function new (string name="adc_item");
            super.new(name);
        endfunction

    endclass

    class adc_sqr extends uvm_sequencer #(adc_item);

        `uvm_component_utils(adc_sqr)

        function new (string name, uvm_component parent);
            super.new(name, parent);
        endfunction

    endclass

    class inject_adc_noise extends uvm_callback;

        real noise[$];
        int fd;
        string str;
        real noise_value;

        `uvm_object_utils(inject_adc_noise)

        function new (string name="");
            super.new(name);
        endfunction

        virtual function void read_noise_file (string file_name);
            fd = $fopen(file_name, "r");
            while ($fscanf(fd,"%s %0f",str,noise_value)==2) begin
                noise.push_back(noise_value);
            end
            $fclose(fd);
        endfunction

        virtual function int convert_noise_to_bits (real noise_val);
            return $rtoi(noise_val * (2 ** (`NBITS - 1) - 1));
        endfunction

        virtual function void inject_noise (adc_item item);
            int noise_dval;
            real nval;
            nval = noise.pop_back();
            noise_dval = convert_noise_to_bits(nval);
            item.data = item.data + adc_resolution'(noise_dval);
        endfunction

        virtual function void shape_noise();
        endfunction

    endclass

    class inject_thermal_noise extends inject_adc_noise;

        `uvm_object_utils(inject_thermal_noise)

        function new (string name="");
            super.new(name);
            read_noise_file("../sv/thermal.dat");
            shape_noise();
        endfunction

        function void inject_noise (adc_item item);
            super.inject_noise(item);
        endfunction

        function void shape_noise();
            foreach (noise[idx]) noise[idx] = noise[idx] * 60.0;
        endfunction

    endclass

    class inject_flicker_noise extends inject_adc_noise;

        `uvm_object_utils(inject_flicker_noise)

        function new (string name="");
            super.new(name);
            read_noise_file("../sv/flicker.dat");
            shape_noise();
        endfunction

        function void inject_noise (adc_item item);
            super.inject_noise(item);
        endfunction

        function void shape_noise();
            foreach (noise[idx]) begin
                if (idx % 2) noise[idx] = 0;
                else noise[idx] = noise[idx] * 60.0;
            end           
        endfunction

    endclass

    class adc_drv extends uvm_driver #(adc_item);

        virtual adc_if m_adc_if;

        `uvm_component_utils(adc_drv)
        `uvm_register_cb(adc_drv,inject_adc_noise)

        function new (string name, uvm_component parent);
            super.new(name, parent);
        endfunction

        function void build_phase (uvm_phase phase);
            super.build_phase(phase);
            if (!uvm_config_db#(virtual adc_if)::get(this,"","m_adc_if",m_adc_if)) begin
                `uvm_error(get_name(),"Failed to get adc_if virtual interface")
            end

        endfunction

        task run_phase(uvm_phase phase);

            wait_for_reset();

            forever begin
                seq_item_port.get_next_item(req);
                begin
                    uvm_callback_iter #(adc_drv, inject_adc_noise) iter = new(this);
                    for (inject_adc_noise cb = iter.first(); cb !=null ; cb = iter.next()) begin
                        cb.inject_noise(req) ;
                    end
                end
                m_adc_if.adc_out <= req.data;
                @(posedge m_adc_if.clk);
                seq_item_port.item_done();
            end
        endtask

        task wait_for_reset();
            @(posedge m_adc_if.rstn);
            @(posedge m_adc_if.clk);
        endtask

    endclass

    class adc_agnt extends uvm_agent;

        adc_sqr m_adc_sqr;
        adc_drv m_adc_drv;

        `uvm_component_utils(adc_agnt)

        function new (string name, uvm_component parent);
            super.new(name, parent);
        endfunction

        function void build_phase (uvm_phase phase);
            super.build_phase(phase);
            m_adc_sqr=adc_sqr::type_id::create("m_adc_sqr", this);
            m_adc_drv=adc_drv::type_id::create("m_adc_drv", this);
        endfunction

        function void connect_phase (uvm_phase phase);
            super.connect_phase(phase);
            m_adc_drv.seq_item_port.connect(m_adc_sqr.seq_item_export);
        endfunction

    endclass

    class adc_seq extends uvm_sequence #(adc_item);

        int cnt;
        int dval;
        real aval;

        `uvm_object_utils(adc_seq)

        function new (string name="");
            super.new(name);
        endfunction

        task body();
            req=adc_item::type_id::create("req");
            repeat (1000) begin
                aval = 0.9 * $sin(2 * `PI * `FREQ * `TSTEP * cnt);
                dval = $rtoi(aval * (2 ** (`NBITS - 1) - 1));
                start_item(req);
                req.data=adc_resolution'(dval);
                finish_item(req);
                cnt++;
            end
        endtask

    endclass

    class cb_iter_test extends uvm_test;

        adc_agnt m_adc_agnt;
        adc_seq m_adc_seq;

        inject_thermal_noise m_inject_thermal_noise;
        inject_flicker_noise m_inject_flicker_noise;

        `uvm_component_utils(cb_iter_test)

        function new (string name, uvm_component parent);
            super.new (name,parent);
        endfunction

        function void build_phase (uvm_phase phase);
            super.build_phase(phase);
            m_adc_agnt=adc_agnt::type_id::create("m_adc_agnt",this);
            m_adc_seq=adc_seq::type_id::create("m_adc_seq",this);
        endfunction

        function void end_of_elaboration_phase (uvm_phase phase);
            m_inject_thermal_noise=inject_thermal_noise::type_id::create("m_inject_thermal_noise",this);
            m_inject_flicker_noise=inject_flicker_noise::type_id::create("m_inject_flicker_noise",this);
            if ($test$plusargs("THERMAL")) begin
                `uvm_info(get_name(),"Add thermal noise to adc output signal",UVM_MEDIUM)
                uvm_callbacks #(adc_drv, inject_adc_noise)::
                add(m_adc_agnt.m_adc_drv, m_inject_thermal_noise);
            end
            if ($test$plusargs("FLICKER")) begin
                `uvm_info(get_name(),"Add flicker noise to adc output signal",UVM_MEDIUM)
                uvm_callbacks #(adc_drv, inject_adc_noise)::
                add(m_adc_agnt.m_adc_drv, m_inject_flicker_noise);
            end
        endfunction

        task run_phase (uvm_phase phase);
            phase.raise_objection(this);
            m_adc_seq.start(m_adc_agnt.m_adc_sqr);
            phase.drop_objection(this);
        endtask

    endclass

endpackage

module top;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import pkg_lib::*;

    logic clk, rstn;

    adc_if m_adc_if(clk,rstn);


    initial begin
        uvm_config_db#(virtual adc_if)::set(null,"uvm_test_top*","m_adc_if",m_adc_if);
        run_test();
    end

    initial begin
        clk <= 1'b1;
        forever #1 clk = ~clk;
    end

    initial begin
        rstn <= 1'b0;
        repeat(2) @ (posedge clk);
        @(negedge clk);
        rstn <= 1'b1;
    end


endmodule
