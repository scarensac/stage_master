< init/gui.tcl

#setup the location of the console and the toolbar

# laptop settings 

set laptop 0

if {$laptop == 1} {
	puts "laptop = 1"
	wm geometry . +1017+0
	console eval {wm geometry . 122x10+0+528}
	launch ControllerEditor 1000 500
	finalizeUI
	wm title .dialog "Control Parameters"
	wm geometry .dialog 260x120+1008+580
} else {
	puts "laptop = 0"
	wm geometry . +1000+0
	console eval {wm geometry . 120x10+0+500}
	launch ControllerEditor 1000 471
	finalizeUI
	wm title .dialog "Control Parameters"
	wm geometry .dialog 260x120+0+0
}

