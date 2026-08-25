# reports_post_route.tcl -- extended post-route report set for the ORB accelerator build.
#
# TWO WAYS TO RUN IT
#   1) As an implementation hook (runs automatically at the end of route_design):
#        add_files -fileset utils_1 <this file>
#        set_property STEPS.ROUTE_DESIGN.TCL.POST <this file> [get_runs impl_1]
#      Vivado stores the hook as $PPRDIR/reports_post_route.tcl, i.e. relative to the
#      project directory, so the project stays relocatable.
#   2) Standalone against an archived routed checkpoint, no synthesis needed:
#        vivado -mode batch -source reports_post_route.tcl -tclargs FAST
#        vivado -mode batch -source reports_post_route.tcl -tclargs HARRIS <path/to.dcp>
#
# WHICH BUILD IS THIS?  Set to FAST or HARRIS to match G_SCORE_TYPE (1 or 0) on the two
# orb_feature_top instances.  Verify afterwards with the DSP count in util_hier:
# 2 total = FAST, 60 total = HARRIS, ~30 = only one instance switched.
# Standalone runs override this with -tclargs, so it only governs the implementation hook.
set CFG HARRIS

# Where archived checkpoints live.  This is the ONE path that cannot be derived: it is a
# different tree from the Vivado project, so it is deliberately left empty here and must be
# supplied per run.  Two ways, in precedence order:
#   1) the 2nd -tclargs value, a full path to the .dcp
#   2) the ARCHIVE_ROOT environment variable, from which the path is built as
#      $ARCHIVE_ROOT/$CFG/HW_Acc_wrapper_routed_$CFG.dcp
# Not needed in hook mode, where a routed design is already in memory.
set ARCHIVE_ROOT ""
if {[info exists ::env(ARCHIVE_ROOT)]} { set ARCHIVE_ROOT $::env(ARCHIVE_ROOT) }

# -tclargs overrides the defaults above when running standalone.
# Guarded three ways because this runs unprotected at the end of a multi-hour route:
# argv may be undefined in some contexts, and in hook mode Vivado launches the run as
# "vivado -mode batch -source <run>.tcl -notrace" with no -tclargs, so anything that did
# leak into argv would start with "-" and must not be mistaken for a config name.
if {[info exists argv] && [llength $argv] > 0 && [string index [lindex $argv 0] 0] ne "-"} {
	set CFG [lindex $argv 0]
}

# --- self-locate: anchor outputs to this script's own directory ----------------
# Works both when sourced as a Vivado hook and under -mode batch -source.
if {[catch {set SCRIPT_DIR [file normalize [file dirname [info script]]]}] || ![file isdirectory $SCRIPT_DIR]} {
	if {[catch {set SCRIPT_DIR [file normalize [get_property DIRECTORY [current_project]]]}]} {
		set SCRIPT_DIR [file normalize [pwd]]
	}
}

# OPEN A CHECKPOINT -- only when no design is already open (standalone mode).
if {[catch {current_design}]} {
	if {[info exists argv] && [llength $argv] > 1} {
		set DCP [file normalize [lindex $argv 1]]
	} elseif {$ARCHIVE_ROOT ne ""} {
		set DCP [file join $ARCHIVE_ROOT $CFG HW_Acc_wrapper_routed_$CFG.dcp]
	} else {
		error "No design in memory and no checkpoint given. Pass the .dcp as the 2nd -tclargs value, or set the ARCHIVE_ROOT environment variable. Example:
    vivado -mode batch -source reports_post_route.tcl -tclargs HARRIS /path/to/HW_Acc_wrapper_routed_HARRIS.dcp"
	}
	puts "INFO: no design in memory, opening $DCP"
	open_checkpoint $DCP
}

# --- CONFIG GUARD ---------------------------------------------------------------
# Make CFG match the design actually in memory before any filename is derived from it.
# harris_response adds ~29 DSP48E2 per accelerator instance, so the dual builds are
# 2 DSPs (FAST) against 60 (HARRIS) device-wide -- an unambiguous discriminator that
# does not depend on the THRESH[16] readback or on remembering to edit CFG.
# TWO WAYS THIS OTHERWISE GOES WRONG: reporting on an already-open design (e.g. after
# open_run impl_1) in standalone mode, or forgetting to edit CFG before a hook run.
# It CORRECTS rather than errors on purpose: raising an error in the ROUTE_DESIGN.TCL.POST
# hook would fail the run before write_bitstream and throw away a multi-hour build over
# a filename.
if {[catch {set n_dsp [llength [get_cells -hier -quiet -filter {REF_NAME =~ "DSP*"}]]} err]} {
	puts "WARNING: config guard could not count DSPs ($err) -- CFG=$CFG is UNVERIFIED"
} else {
	if {$n_dsp > 10} { set detected HARRIS } else { set detected FAST }
	if {$detected ne $CFG} {
		puts "WARNING: CONFIG MISMATCH -- CFG was '$CFG' but the design in memory has $n_dsp"
		puts "WARNING: DSP cells, i.e. '$detected'.  Overriding CFG to '$detected' so the"
		puts "WARNING: reports are not mislabelled.  Check which design you meant to report on."
		set CFG $detected
	} else {
		puts "INFO: config guard OK -- CFG=$CFG, $n_dsp DSP cells in design"
	}
}

set OUT [file join $SCRIPT_DIR _reports $CFG]
file mkdir $OUT

puts "INFO: config=$CFG  script_dir=$SCRIPT_DIR  out=$OUT"

# Every report is wrapped so that a failing command can never fail the parent run.
proc rpt {label body} {
	if {[catch {uplevel 1 $body} err]} {
		puts "WARNING: report '$label' failed: $err"
	} else {
		puts "INFO: report '$label' written"
	}
}

# --- per-module resource cost (feeds the Ch4 module table) ---------------------
# Depth must reach the accelerator internals.  The path to the deepest module of interest is
# HW_Acc_wrapper/HW_Acc_i/orb_feature_top_0/U0/u_top/u_core/orient/gen_harris.hr = 8 levels,
# so depth 4 truncates at u_top and hides every module the Ch4 table needs.
rpt util_hier  {report_utilization -hierarchical -hierarchical_depth 12 -hierarchical_percentages -file [file join $OUT util_hier_$CFG.rpt]}
rpt ram_util   {report_ram_utilization -file [file join $OUT ram_util_$CFG.rpt]}

# --- where the delay actually goes --------------------------------------------
rpt da_timing  {report_design_analysis -timing -setup -max_paths 25 -show_all -file [file join $OUT da_timing_$CFG.rpt]}
rpt da_levels  {report_design_analysis -logic_level_distribution -logic_level_dist_paths 1000 -file [file join $OUT da_levels_$CFG.rpt]}
rpt da_congest {report_design_analysis -congestion -complexity -hierarchical_depth 2 -file [file join $OUT da_congestion_$CFG.rpt]}
# NOTE: -histogram and -max_nets are mutually exclusive (Common 17-69), hence two calls.
rpt fanout     {report_high_fanout_nets -max_nets 50 -timing -file [file join $OUT high_fanout_$CFG.rpt]}
rpt fanout_his {report_high_fanout_nets -histogram -file [file join $OUT high_fanout_histogram_$CFG.rpt]}

# --- population of near-critical paths, not just the single worst one ----------
rpt near_crit  {report_timing -setup -max_paths 200 -nworst 200 -slack_lesser_than 1.0 -file [file join $OUT timing_near_critical_$CFG.rpt]}

# --- single-clock-domain evidence (expected to be near-empty, that is the point)
rpt clk_inter  {report_clock_interaction -file [file join $OUT clock_interaction_$CFG.rpt]}

# --- power: vectorless, keep it labelled as such -------------------------------
rpt power_adv  {report_power -advisory -file [file join $OUT power_advisory_$CFG.rpt]}

puts "INFO: extended report set for $CFG complete -> $OUT"
