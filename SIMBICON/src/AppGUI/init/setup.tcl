< ../AppGUI/init/gui.tcl

#setup the location of the console and the toolbar

# laptop settings 
#laptop 0 is the evolution interface (no console)
set laptop 0

if {$laptop == 1} {
	puts "laptop = 1"
	wm geometry . +1000+0
	console eval {wm geometry . 120x10+0+500}
	launch ControllerEditor 1000 450
	finalizeUI
	
} else {
	wm geometry . +1000+0
	launch ControllerEditor 1000 450
	finalizeUI
}

