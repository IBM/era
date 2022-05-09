; ModuleID = '../src/main.c'
source_filename = "../src/main.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

%struct.Observation = type { i8, %struct.geometry_msgs_Point, float, double, double, double, %struct.Costmap2D_struct }
%struct.geometry_msgs_Point = type { double, double, double }
%struct.Costmap2D_struct = type { double, double, double, double, i32, i32, i8, [2500 x i8] }
%struct.timeval = type { i64, i64 }
%struct._IO_FILE = type { i32, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, %struct._IO_marker*, %struct._IO_FILE*, i32, i32, i64, i16, i8, [1 x i8], i8*, i64, i8*, i8*, i8*, i8*, i64, i32, [20 x i8] }
%struct._IO_marker = type { %struct._IO_marker*, %struct._IO_FILE*, i32 }
%struct.lidar_inputs_struct = type { [3 x float], i32, [200002 x i8] }
%struct.sockaddr_in = type { i16, i16, %struct.in_addr, [8 x i8] }
%struct.in_addr = type { i32 }
%struct.sockaddr = type { i16, [14 x i8] }
%struct.timezone = type { i32, i32 }

@.str = private unnamed_addr constant [16 x i8] c"gridimage_era1_\00", align 1
@IMAGE_FN = dso_local local_unnamed_addr global i8* getelementptr inbounds ([16 x i8], [16 x i8]* @.str, i64 0, i64 0), align 8
@max_time_steps = dso_local local_unnamed_addr global i32 -2, align 4
@bag_sock = dso_local local_unnamed_addr global i32 0, align 4
@xmit_sock = dso_local local_unnamed_addr global i32 0, align 4
@recv_sock = dso_local local_unnamed_addr global i32 0, align 4
@car_sock = dso_local local_unnamed_addr global i32 0, align 4
@.str.1 = private unnamed_addr constant [3 x i8] c"OK\00", align 1
@ack = dso_local local_unnamed_addr global i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str.1, i64 0, i64 0), align 8
@odometry = dso_local local_unnamed_addr global [3 x float] zeroinitializer, align 4
@curr_obs = dso_local local_unnamed_addr global i32 1, align 4
@next_obs = dso_local local_unnamed_addr global i32 0, align 4
@proc_odo_sec = dso_local local_unnamed_addr global i64 0, align 8
@proc_odo_usec = dso_local local_unnamed_addr global i64 0, align 8
@proc_rdbag_sec = dso_local local_unnamed_addr global i64 0, align 8
@proc_rdbag_usec = dso_local local_unnamed_addr global i64 0, align 8
@proc_lidar_sec = dso_local local_unnamed_addr global i64 0, align 8
@proc_lidar_usec = dso_local local_unnamed_addr global i64 0, align 8
@proc_data_sec = dso_local local_unnamed_addr global i64 0, align 8
@proc_data_usec = dso_local local_unnamed_addr global i64 0, align 8
@proc_cv_sec = dso_local local_unnamed_addr global i64 0, align 8
@proc_cv_usec = dso_local local_unnamed_addr global i64 0, align 8
@counter = dso_local local_unnamed_addr global i32 0, align 4
@ascii_counter = dso_local local_unnamed_addr global i32 0, align 4
@odo_count = dso_local local_unnamed_addr global i32 0, align 4
@lidar_count = dso_local local_unnamed_addr global i32 0, align 4
@lmap_count = dso_local local_unnamed_addr global i32 0, align 4
@xmit_count = dso_local local_unnamed_addr global i32 0, align 4
@recv_count = dso_local local_unnamed_addr global i32 0, align 4
@car_send_count = dso_local local_unnamed_addr global i32 0, align 4
@cv_count = dso_local local_unnamed_addr global i32 0, align 4
@.str.2 = private unnamed_addr constant [21 x i8] c"Usage: %s <OPTIONS>\0A\00", align 1
@.str.10 = private unnamed_addr constant [21 x i8] c"Received a SIGINT...\00", align 1
@.str.12 = private unnamed_addr constant [22 x i8] c"Received a SIGPIPE...\00", align 1
@.str.13 = private unnamed_addr constant [60 x i8] c"closeout_and_exit -- Closing the connection and exiting %d\0A\00", align 1
@.str.15 = private unnamed_addr constant [11 x i8] c"%s%04d.ppm\00", align 1
@.str.16 = private unnamed_addr constant [2 x i8] c"w\00", align 1
@.str.17 = private unnamed_addr constant [14 x i8] c"P3 %d %d 255\0A\00", align 1
@.str.18 = private unnamed_addr constant [11 x i8] c" %d %d %d \00", align 1
@.str.20 = private unnamed_addr constant [27 x i8] c"Unexpected WiFi message...\00", align 1
@.str.22 = private unnamed_addr constant [27 x i8] c"RECV REAL got zero bytes..\00", align 1
@.str.23 = private unnamed_addr constant [64 x i8] c"  RECV REAL got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\0A\00", align 1
@.str.24 = private unnamed_addr constant [30 x i8] c"RECV REAL got too few bytes..\00", align 1
@.str.26 = private unnamed_addr constant [27 x i8] c"RECV IMAG got zero bytes..\00", align 1
@.str.27 = private unnamed_addr constant [64 x i8] c"  RECV IMAG got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\0A\00", align 1
@.str.28 = private unnamed_addr constant [30 x i8] c"RECV IMAG got too few bytes..\00", align 1
@.str.29 = private unnamed_addr constant [32 x i8] c"LZ4_decompress_safe ERROR : %d\0A\00", align 1
@.str.30 = private unnamed_addr constant [55 x i8] c"Receive step %u : Processing fusion for curr_obs = %d\0A\00", align 1
@.str.31 = private unnamed_addr constant [7 x i8] c"X%-6uX\00", align 1
@observationsArr = common dso_local global [2 x %struct.Observation] zeroinitializer, align 16
@bag_inet_addr_str = common dso_local global [20 x i8] zeroinitializer, align 16
@.str.32 = private unnamed_addr constant [10 x i8] c"127.0.0.1\00", align 1
@wifi_inet_addr_str = common dso_local global [20 x i8] zeroinitializer, align 16
@car_inet_addr_str = common dso_local global [20 x i8] zeroinitializer, align 16
@.str.38 = private unnamed_addr constant [11 x i8] c":hB:W:C:s:\00", align 1
@.str.39 = private unnamed_addr constant [3 x i8] c"%s\00", align 1
@optarg = external dso_local local_unnamed_addr global i8*, align 8
@.str.41 = private unnamed_addr constant [20 x i8] c"unknown option: %c\0A\00", align 1
@optopt = external dso_local local_unnamed_addr global i32, align 4
@.str.42 = private unnamed_addr constant [31 x i8] c"Limiting to %u max time steps\0A\00", align 1
@.str.44 = private unnamed_addr constant [43 x i8] c"Connecting to bag-server at IP %s PORT %u\0A\00", align 1
@.str.49 = private unnamed_addr constant [44 x i8] c"Connecting to xmit-server at IP %s PORT %u\0A\00", align 1
@.str.54 = private unnamed_addr constant [44 x i8] c"Connecting to recv-server at IP %s PORT %u\0A\00", align 1
@.str.59 = private unnamed_addr constant [43 x i8] c"Connecting to car-server at IP %s PORT %u\0A\00", align 1
@start_prog = common dso_local global %struct.timeval zeroinitializer, align 8
@start_proc_rdbag = common dso_local global %struct.timeval zeroinitializer, align 8
@stop_proc_rdbag = common dso_local global %struct.timeval zeroinitializer, align 8
@.str.64 = private unnamed_addr constant [50 x i8] c"  TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\0A\00", align 1
@.str.65 = private unnamed_addr constant [30 x i8] c"Top read got too few bytes...\00", align 1
@start_proc_lidar = common dso_local global %struct.timeval zeroinitializer, align 8
@.str.67 = private unnamed_addr constant [29 x i8] c"Lidar read got zero bytes...\00", align 1
@.str.68 = private unnamed_addr constant [65 x i8] c"  Lidar read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\0A\00", align 1
@.str.69 = private unnamed_addr constant [32 x i8] c"Lidar read got too few bytes...\00", align 1
@.str.70 = private unnamed_addr constant [53 x i8] c"NOTE: read more total bytes than expected: %u vs %u\0A\00", align 1
@.str.71 = private unnamed_addr constant [31 x i8] c"Processing Lidar msg %4u data\0A\00", align 1
@start_proc_data = common dso_local global %struct.timeval zeroinitializer, align 8
@stop_proc_lidar = common dso_local global %struct.timeval zeroinitializer, align 8
@.str.73 = private unnamed_addr constant [32 x i8] c"Odometry read got zero bytes...\00", align 1
@.str.74 = private unnamed_addr constant [63 x i8] c"  Odo read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\0A\00", align 1
@.str.75 = private unnamed_addr constant [35 x i8] c"Odometry read got too few bytes...\00", align 1
@.str.76 = private unnamed_addr constant [34 x i8] c"Odometry msg %4u: %.2f %.2f %.2f\0A\00", align 1
@start_proc_cv = common dso_local global %struct.timeval zeroinitializer, align 8
@stop_proc_cv = common dso_local global %struct.timeval zeroinitializer, align 8
@.str.77 = private unnamed_addr constant [44 x i8] c"run_object_classification time in usec %ld\0A\00", align 1
@.str.78 = private unnamed_addr constant [39 x i8] c"run_object_classification returned %u\0A\00", align 1
@.str.79 = private unnamed_addr constant [90 x i8] c"\0AFinal Run Stats, %u, Odo, %u, Lidar, %u, LMAP, %u, XMIT, %u, RECV, %u, CAR-SEND, %u, CV\0A\00", align 1
@.str.80 = private unnamed_addr constant [59 x i8] c"Occ-Map Dimensions, %u, by, %u, grid, res, %lf, ray_r, %u\0A\00", align 1
@.str.81 = private unnamed_addr constant [18 x i8] c"Timing (in usec):\00", align 1
@.str.82 = private unnamed_addr constant [16 x i8] c" with NO HW_VIT\00", align 1
@.str.83 = private unnamed_addr constant [20 x i8] c" and NO HW_XMIT_FFT\00", align 1
@.str.84 = private unnamed_addr constant [20 x i8] c" and NO HW_RECV_FFT\00", align 1
@stop_prog = common dso_local global %struct.timeval zeroinitializer, align 8
@.str.86 = private unnamed_addr constant [40 x i8] c" Total workload main-loop : %10lu usec\0A\00", align 1
@.str.87 = private unnamed_addr constant [42 x i8] c"   Total proc Read-Bag      : %10lu usec\0A\00", align 1
@.str.88 = private unnamed_addr constant [42 x i8] c"   Total proc Odometry      : %10lu usec\0A\00", align 1
@.str.89 = private unnamed_addr constant [42 x i8] c"   Total proc Lidar         : %10lu usec\0A\00", align 1
@.str.90 = private unnamed_addr constant [44 x i8] c"     Total proc Data          : %10lu usec\0A\00", align 1
@.str.91 = private unnamed_addr constant [42 x i8] c"     Total proc CV          : %10lu usec\0A\00", align 1
@stop_proc_odo = common dso_local local_unnamed_addr global %struct.timeval zeroinitializer, align 8
@start_proc_odo = common dso_local local_unnamed_addr global %struct.timeval zeroinitializer, align 8
@stop_proc_data = common dso_local local_unnamed_addr global %struct.timeval zeroinitializer, align 8
@str = private unnamed_addr constant [10 x i8] c" OPTIONS:\00", align 1
@str.94 = private unnamed_addr constant [47 x i8] c"    -h         : print this helpful usage info\00", align 1
@str.95 = private unnamed_addr constant [74 x i8] c"    -B <str>   : set the internet-address for the bagfile server to <str>\00", align 1
@str.96 = private unnamed_addr constant [71 x i8] c"    -W <str>   : set the internet-address for the WiFi server to <str>\00", align 1
@str.97 = private unnamed_addr constant [81 x i8] c"    -C <str>   : set the internet-address for the Car Map Output server to <str>\00", align 1
@str.98 = private unnamed_addr constant [62 x i8] c"    -s <Num>   : exit run after <Num> Lidar time-steps (msgs)\00", align 1
@str.99 = private unnamed_addr constant [59 x i8] c"In SIGINT INThandler -- Closing the connection and exiting\00", align 1
@str.100 = private unnamed_addr constant [57 x i8] c"In SIGPIPE_handler -- Closing the connection and exiting\00", align 1
@str.101 = private unnamed_addr constant [39 x i8] c"ERROR: Unexpected message from WiFi...\00", align 1
@str.102 = private unnamed_addr constant [47 x i8] c"  RECV IMAG got ZERO bytes -- END of TRANSFER?\00", align 1
@str.103 = private unnamed_addr constant [47 x i8] c"  RECV REAL got ZERO bytes -- END of TRANSFER?\00", align 1
@str.104 = private unnamed_addr constant [34 x i8] c"Initializing the OccGrid state...\00", align 1
@str.105 = private unnamed_addr constant [38 x i8] c"Initializing the Transmit pipeline...\00", align 1
@str.106 = private unnamed_addr constant [37 x i8] c"Initializing the Receive pipeline...\00", align 1
@str.107 = private unnamed_addr constant [44 x i8] c"Initializing the Computer Vision toolset...\00", align 1
@str.108 = private unnamed_addr constant [29 x i8] c"Running the entire bag file.\00", align 1
@str.109 = private unnamed_addr constant [34 x i8] c"BAG Socket successfully created..\00", align 1
@str.110 = private unnamed_addr constant [30 x i8] c"connected to the BAG server..\00", align 1
@str.111 = private unnamed_addr constant [40 x i8] c"WIFI XMIT Socket successfully created..\00", align 1
@str.112 = private unnamed_addr constant [36 x i8] c"connected to the WIFI XMIT server..\00", align 1
@str.113 = private unnamed_addr constant [40 x i8] c"WIFI RECV Socket successfully created..\00", align 1
@str.114 = private unnamed_addr constant [36 x i8] c"connected to the WIFI RECV server..\00", align 1
@str.115 = private unnamed_addr constant [34 x i8] c"CAR Socket successfully created..\00", align 1
@str.116 = private unnamed_addr constant [30 x i8] c"connected to the CAR server..\00", align 1
@str.117 = private unnamed_addr constant [46 x i8] c"  Odo read got ZERO bytes -- END of TRANSFER?\00", align 1
@str.118 = private unnamed_addr constant [48 x i8] c"  Lidar read got ZERO bytes -- END of TRANSFER?\00", align 1
@str.119 = private unnamed_addr constant [41 x i8] c"connection with the CAR server failed...\00", align 1
@str.120 = private unnamed_addr constant [30 x i8] c"CAR Socket creation failed...\00", align 1
@str.121 = private unnamed_addr constant [47 x i8] c"connection with the WIFI RECV server failed...\00", align 1
@str.122 = private unnamed_addr constant [36 x i8] c"WIFI RECV Socket creation failed...\00", align 1
@str.123 = private unnamed_addr constant [47 x i8] c"connection with the WIFI XMIT server failed...\00", align 1
@str.124 = private unnamed_addr constant [36 x i8] c"WIFI XMIT Socket creation failed...\00", align 1
@str.125 = private unnamed_addr constant [41 x i8] c"connection with the BAG server failed...\00", align 1
@str.126 = private unnamed_addr constant [30 x i8] c"BAG Socket creation failed...\00", align 1
@str.127 = private unnamed_addr constant [21 x i8] c"option needs a value\00", align 1
@str.128 = private unnamed_addr constant [49 x i8] c"Computer Vision toolset initialization failed...\00", align 1
@str.129 = private unnamed_addr constant [52 x i8] c" NO more detailed timing information on this run...\00", align 1
@str.130 = private unnamed_addr constant [22 x i8] c"\0ADone with the run...\00", align 1

; Function Attrs: nofree nounwind uwtable
define dso_local void @print_usage(i8* %pname) local_unnamed_addr #0 {
entry:
  %call = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([21 x i8], [21 x i8]* @.str.2, i64 0, i64 0), i8* %pname)
  %puts = tail call i32 @puts(i8* getelementptr inbounds ([10 x i8], [10 x i8]* @str, i64 0, i64 0))
  %puts7 = tail call i32 @puts(i8* getelementptr inbounds ([47 x i8], [47 x i8]* @str.94, i64 0, i64 0))
  %puts8 = tail call i32 @puts(i8* getelementptr inbounds ([74 x i8], [74 x i8]* @str.95, i64 0, i64 0))
  %puts9 = tail call i32 @puts(i8* getelementptr inbounds ([71 x i8], [71 x i8]* @str.96, i64 0, i64 0))
  %puts10 = tail call i32 @puts(i8* getelementptr inbounds ([81 x i8], [81 x i8]* @str.97, i64 0, i64 0))
  %puts11 = tail call i32 @puts(i8* getelementptr inbounds ([62 x i8], [62 x i8]* @str.98, i64 0, i64 0))
  ret void
}

; Function Attrs: nofree nounwind
declare dso_local i32 @printf(i8* nocapture readonly, ...) local_unnamed_addr #1

; Function Attrs: noreturn nounwind uwtable
define dso_local void @INThandler(i32 %dummy) #2 {
entry:
  %puts = tail call i32 @puts(i8* getelementptr inbounds ([59 x i8], [59 x i8]* @str.99, i64 0, i64 0))
  tail call void @closeout_and_exit(i8* getelementptr inbounds ([21 x i8], [21 x i8]* @.str.10, i64 0, i64 0), i32 -1)
  unreachable
}

; Function Attrs: noreturn nounwind uwtable
define dso_local void @closeout_and_exit(i8* nocapture readonly %last_msg, i32 %rval) local_unnamed_addr #2 {
entry:
  %0 = load i32, i32* @lidar_count, align 4, !tbaa !2
  %cmp = icmp eq i32 %0, 0
  br i1 %cmp, label %if.end, label %if.then

if.then:                                          ; preds = %entry
  tail call void @dump_final_run_statistics()
  br label %if.end

if.end:                                           ; preds = %entry, %if.then
  %call = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([60 x i8], [60 x i8]* @.str.13, i64 0, i64 0), i32 %rval)
  %1 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %cmp1 = icmp eq i32 %1, 0
  br i1 %cmp1, label %if.end4, label %if.then2

if.then2:                                         ; preds = %if.end
  %call3 = tail call i32 @close(i32 %1) #11
  br label %if.end4

if.end4:                                          ; preds = %if.end, %if.then2
  %2 = load i32, i32* @xmit_sock, align 4, !tbaa !2
  %cmp5 = icmp eq i32 %2, 0
  br i1 %cmp5, label %if.end8, label %if.then6

if.then6:                                         ; preds = %if.end4
  %call7 = tail call i32 @close(i32 %2) #11
  br label %if.end8

if.end8:                                          ; preds = %if.end4, %if.then6
  %3 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %cmp9 = icmp eq i32 %3, 0
  br i1 %cmp9, label %if.end12, label %if.then10

if.then10:                                        ; preds = %if.end8
  %call11 = tail call i32 @close(i32 %3) #11
  br label %if.end12

if.end12:                                         ; preds = %if.end8, %if.then10
  %4 = load i32, i32* @car_sock, align 4, !tbaa !2
  %cmp13 = icmp eq i32 %4, 0
  br i1 %cmp13, label %if.end16, label %if.then14

if.then14:                                        ; preds = %if.end12
  %call15 = tail call i32 @close(i32 %4) #11
  br label %if.end16

if.end16:                                         ; preds = %if.end12, %if.then14
  %puts = tail call i32 @puts(i8* %last_msg)
  tail call void @exit(i32 %rval) #12
  unreachable
}

; Function Attrs: noreturn nounwind uwtable
define dso_local void @SIGPIPE_handler(i32 %dummy) #2 {
entry:
  %puts = tail call i32 @puts(i8* getelementptr inbounds ([57 x i8], [57 x i8]* @str.100, i64 0, i64 0))
  tail call void @closeout_and_exit(i8* getelementptr inbounds ([22 x i8], [22 x i8]* @.str.12, i64 0, i64 0), i32 -1)
  unreachable
}

declare dso_local i32 @close(i32) local_unnamed_addr #3

; Function Attrs: noreturn nounwind
declare dso_local void @exit(i32) local_unnamed_addr #4

; Function Attrs: nounwind uwtable
define dso_local void @write_array_to_file(i8* nocapture readonly %data, i64 %size) local_unnamed_addr #5 {
entry:
  %file_name = alloca [32 x i8], align 16
  %0 = getelementptr inbounds [32 x i8], [32 x i8]* %file_name, i64 0, i64 0
  call void @llvm.lifetime.start.p0i8(i64 32, i8* nonnull %0) #11
  %1 = load i8*, i8** @IMAGE_FN, align 8, !tbaa !6
  %2 = load i32, i32* @counter, align 4, !tbaa !2
  %call = call i32 (i8*, i64, i8*, ...) @snprintf(i8* nonnull %0, i64 32, i8* getelementptr inbounds ([11 x i8], [11 x i8]* @.str.15, i64 0, i64 0), i8* %1, i32 %2) #11
  %call2 = call %struct._IO_FILE* @fopen(i8* nonnull %0, i8* getelementptr inbounds ([2 x i8], [2 x i8]* @.str.16, i64 0, i64 0))
  %call3 = tail call i32 (%struct._IO_FILE*, i8*, ...) @fprintf(%struct._IO_FILE* %call2, i8* getelementptr inbounds ([14 x i8], [14 x i8]* @.str.17, i64 0, i64 0), i32 50, i32 50)
  br label %for.body

for.body:                                         ; preds = %for.body, %entry
  %indvars.iv = phi i64 [ 0, %entry ], [ %indvars.iv.next, %for.body ]
  %arrayidx = getelementptr inbounds i8, i8* %data, i64 %indvars.iv
  %3 = load i8, i8* %arrayidx, align 1, !tbaa !8
  %conv = zext i8 %3 to i32
  %call10 = tail call i32 (%struct._IO_FILE*, i8*, ...) @fprintf(%struct._IO_FILE* %call2, i8* getelementptr inbounds ([11 x i8], [11 x i8]* @.str.18, i64 0, i64 0), i32 %conv, i32 %conv, i32 %conv)
  %indvars.iv.next = add nuw nsw i64 %indvars.iv, 1
  %exitcond = icmp eq i64 %indvars.iv.next, 2500
  br i1 %exitcond, label %for.end, label %for.body

for.end:                                          ; preds = %for.body
  %call11 = tail call i32 @fclose(%struct._IO_FILE* %call2)
  %4 = load i32, i32* @counter, align 4, !tbaa !2
  %inc12 = add nsw i32 %4, 1
  store i32 %inc12, i32* @counter, align 4, !tbaa !2
  call void @llvm.lifetime.end.p0i8(i64 32, i8* nonnull %0) #11
  ret void
}

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.start.p0i8(i64 immarg, i8* nocapture) #6

; Function Attrs: nofree nounwind
declare dso_local i32 @snprintf(i8* nocapture, i64, i8* nocapture readonly, ...) local_unnamed_addr #1

; Function Attrs: nofree nounwind
declare dso_local noalias %struct._IO_FILE* @fopen(i8* nocapture readonly, i8* nocapture readonly) local_unnamed_addr #1

; Function Attrs: nofree nounwind
declare dso_local i32 @fprintf(%struct._IO_FILE* nocapture, i8* nocapture readonly, ...) local_unnamed_addr #1

; Function Attrs: nofree nounwind
declare dso_local i32 @fclose(%struct._IO_FILE* nocapture) local_unnamed_addr #1

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.end.p0i8(i64 immarg, i8* nocapture) #6

; Function Attrs: nofree nounwind uwtable
define dso_local i32 @read_all(i32 %sock, i8* nocapture %buffer, i32 %xfer_in_bytes) local_unnamed_addr #0 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.body, %entry
  %total_recvd.0 = phi i32 [ 0, %entry ], [ %add, %while.body ]
  %message_ptr.0 = phi i8* [ %buffer, %entry ], [ %add.ptr, %while.body ]
  %cmp = icmp slt i32 %total_recvd.0, %xfer_in_bytes
  br i1 %cmp, label %while.body, label %cleanup5

while.body:                                       ; preds = %while.cond
  %sub = sub nsw i32 %xfer_in_bytes, %total_recvd.0
  %conv = zext i32 %sub to i64
  %call = tail call i64 @read(i32 %sock, i8* %message_ptr.0, i64 %conv) #11
  %conv1 = trunc i64 %call to i32
  %sext = shl i64 %call, 32
  %idx.ext = ashr exact i64 %sext, 32
  %add.ptr = getelementptr inbounds i8, i8* %message_ptr.0, i64 %idx.ext
  %add = add nsw i32 %total_recvd.0, %conv1
  %cmp2 = icmp eq i32 %conv1, 0
  br i1 %cmp2, label %cleanup5, label %while.cond

cleanup5:                                         ; preds = %while.body, %while.cond
  ret i32 %total_recvd.0
}

; Function Attrs: nofree
declare dso_local i64 @read(i32, i8* nocapture, i64) local_unnamed_addr #7

; Function Attrs: nounwind uwtable
define dso_local noalias i8* @receive_and_fuse_maps_impl(%struct.Observation* %observations, i64 %observations_sz) local_unnamed_addr #5 {
entry:
  %recvd_in_real = alloca [41800 x float], align 16
  %recvd_in_imag = alloca [41800 x float], align 16
  %r_buffer = alloca [10 x i8], align 1
  %ptr = alloca i8*, align 8
  %recvd_msg_len_inner = alloca i32, align 4
  %recvd_msg = alloca [1500 x i8], align 16
  %uncmp_data = alloca %struct.Costmap2D_struct, align 16
  %scrambled_msg = alloca [18585 x i8], align 16
  %ss_freq_offset = alloca float, align 4
  %num_sync_short_vals = alloca i32, align 4
  %sl_freq_offset = alloca float, align 4
  %num_sync_long_vals = alloca i32, align 4
  %fft_ar_r = alloca [33280 x float], align 16
  %fft_ar_i = alloca [33280 x float], align 16
  %num_fft_outs = alloca i32, align 4
  %toBeEqualized = alloca [33280 x { float, float }], align 16
  %equalized = alloca [24960 x { float, float }], align 16
  %num_eq_out_bits = alloca i32, align 4
  %psdu = alloca i32, align 4
  %local_map = alloca %struct.Costmap2D_struct*, align 8
  %0 = bitcast [41800 x float]* %recvd_in_real to i8*
  call void @llvm.lifetime.start.p0i8(i64 167200, i8* nonnull %0) #11
  %1 = bitcast [41800 x float]* %recvd_in_imag to i8*
  call void @llvm.lifetime.start.p0i8(i64 167200, i8* nonnull %1) #11
  %2 = getelementptr inbounds [10 x i8], [10 x i8]* %r_buffer, i64 0, i64 0
  call void @llvm.lifetime.start.p0i8(i64 10, i8* nonnull %2) #11
  %3 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %call = call i32 @read_all(i32 %3, i8* nonnull %2, i32 8)
  %cmp = icmp eq i32 %call, 8
  br i1 %cmp, label %if.then, label %if.end69

if.then:                                          ; preds = %entry
  %4 = load i8, i8* %2, align 1, !tbaa !8
  %cmp1 = icmp eq i8 %4, 88
  br i1 %cmp1, label %land.lhs.true, label %if.then7

land.lhs.true:                                    ; preds = %if.then
  %arrayidx3 = getelementptr inbounds [10 x i8], [10 x i8]* %r_buffer, i64 0, i64 7
  %5 = load i8, i8* %arrayidx3, align 1, !tbaa !8
  %cmp5 = icmp eq i8 %5, 88
  br i1 %cmp5, label %if.end, label %if.then7

if.then7:                                         ; preds = %land.lhs.true, %if.then
  %puts = tail call i32 @puts(i8* getelementptr inbounds ([39 x i8], [39 x i8]* @str.101, i64 0, i64 0))
  tail call void @closeout_and_exit(i8* getelementptr inbounds ([27 x i8], [27 x i8]* @.str.20, i64 0, i64 0), i32 -3)
  unreachable

if.end:                                           ; preds = %land.lhs.true
  %6 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %7 = load i8*, i8** @ack, align 8, !tbaa !6
  %call9 = tail call i64 @send(i32 %6, i8* %7, i64 2, i32 0) #11
  %8 = bitcast i8** %ptr to i8*
  call void @llvm.lifetime.start.p0i8(i64 8, i8* nonnull %8) #11
  %add.ptr = getelementptr inbounds [10 x i8], [10 x i8]* %r_buffer, i64 0, i64 1
  %call11 = call i64 @strtol(i8* nonnull %add.ptr, i8** nonnull %ptr, i32 10) #11
  %conv12 = trunc i64 %call11 to i32
  %conv13 = lshr i64 %call11, 2
  %9 = trunc i64 %conv13 to i32
  %conv14 = and i32 %9, 1073741823
  %10 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %arraydecay15 = getelementptr inbounds [41800 x float], [41800 x float]* %recvd_in_real, i64 0, i64 0
  %call16 = call i32 @read_all(i32 %10, i8* nonnull %0, i32 %conv12)
  %cmp17 = icmp ult i32 %call16, %conv12
  br i1 %cmp17, label %if.then19, label %if.end26

if.then19:                                        ; preds = %if.end
  %cmp20 = icmp eq i32 %call16, 0
  br i1 %cmp20, label %if.then22, label %if.else

if.then22:                                        ; preds = %if.then19
  %puts121 = call i32 @puts(i8* getelementptr inbounds ([47 x i8], [47 x i8]* @str.103, i64 0, i64 0))
  call void @closeout_and_exit(i8* getelementptr inbounds ([27 x i8], [27 x i8]* @.str.22, i64 0, i64 0), i32 -1)
  unreachable

if.else:                                          ; preds = %if.then19
  %call24 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([64 x i8], [64 x i8]* @.str.23, i64 0, i64 0), i32 %call16, i32 %conv12)
  call void @closeout_and_exit(i8* getelementptr inbounds ([30 x i8], [30 x i8]* @.str.24, i64 0, i64 0), i32 -1)
  unreachable

if.end26:                                         ; preds = %if.end
  %11 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %call28 = call i32 @read_all(i32 %11, i8* nonnull %1, i32 %conv12)
  %cmp29 = icmp ult i32 %call28, %conv12
  br i1 %cmp29, label %if.then31, label %if.end39

if.then31:                                        ; preds = %if.end26
  %cmp32 = icmp eq i32 %call28, 0
  br i1 %cmp32, label %if.then34, label %if.else36

if.then34:                                        ; preds = %if.then31
  %puts120 = call i32 @puts(i8* getelementptr inbounds ([47 x i8], [47 x i8]* @str.102, i64 0, i64 0))
  call void @closeout_and_exit(i8* getelementptr inbounds ([27 x i8], [27 x i8]* @.str.26, i64 0, i64 0), i32 -1)
  unreachable

if.else36:                                        ; preds = %if.then31
  %call37 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([64 x i8], [64 x i8]* @.str.27, i64 0, i64 0), i32 %call28, i32 %conv12)
  call void @closeout_and_exit(i8* getelementptr inbounds ([30 x i8], [30 x i8]* @.str.28, i64 0, i64 0), i32 -1)
  unreachable

if.end39:                                         ; preds = %if.end26
  %arraydecay27 = getelementptr inbounds [41800 x float], [41800 x float]* %recvd_in_imag, i64 0, i64 0
  %12 = bitcast i32* %recvd_msg_len_inner to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %12) #11
  store i32 0, i32* %recvd_msg_len_inner, align 4, !tbaa !2
  %13 = getelementptr inbounds [1500 x i8], [1500 x i8]* %recvd_msg, i64 0, i64 0
  call void @llvm.lifetime.start.p0i8(i64 1500, i8* nonnull %13) #11
  %14 = bitcast %struct.Costmap2D_struct* %uncmp_data to i8*
  call void @llvm.lifetime.start.p0i8(i64 2544, i8* nonnull %14) #11
  %15 = getelementptr inbounds [18585 x i8], [18585 x i8]* %scrambled_msg, i64 0, i64 0
  call void @llvm.lifetime.start.p0i8(i64 18585, i8* nonnull %15) #11
  %16 = bitcast float* %ss_freq_offset to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %16) #11
  store float 0.000000e+00, float* %ss_freq_offset, align 4, !tbaa !9
  %17 = bitcast i32* %num_sync_short_vals to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %17) #11
  store i32 0, i32* %num_sync_short_vals, align 4, !tbaa !2
  %18 = bitcast float* %sl_freq_offset to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %18) #11
  store float 0.000000e+00, float* %sl_freq_offset, align 4, !tbaa !9
  %19 = bitcast i32* %num_sync_long_vals to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %19) #11
  store i32 0, i32* %num_sync_long_vals, align 4, !tbaa !2
  %20 = bitcast [33280 x float]* %fft_ar_r to i8*
  call void @llvm.lifetime.start.p0i8(i64 133120, i8* nonnull %20) #11
  %21 = bitcast [33280 x float]* %fft_ar_i to i8*
  call void @llvm.lifetime.start.p0i8(i64 133120, i8* nonnull %21) #11
  %22 = bitcast i32* %num_fft_outs to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %22) #11
  store i32 0, i32* %num_fft_outs, align 4, !tbaa !2
  %23 = bitcast [33280 x { float, float }]* %toBeEqualized to i8*
  call void @llvm.lifetime.start.p0i8(i64 266240, i8* nonnull %23) #11
  %24 = bitcast [24960 x { float, float }]* %equalized to i8*
  call void @llvm.lifetime.start.p0i8(i64 199680, i8* nonnull %24) #11
  %25 = bitcast i32* %num_eq_out_bits to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %25) #11
  store i32 0, i32* %num_eq_out_bits, align 4, !tbaa !2
  %26 = bitcast i32* %psdu to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %26) #11
  store i32 0, i32* %psdu, align 4, !tbaa !2
  %arraydecay44 = getelementptr inbounds [33280 x float], [33280 x float]* %fft_ar_r, i64 0, i64 0
  %arraydecay45 = getelementptr inbounds [33280 x float], [33280 x float]* %fft_ar_i, i64 0, i64 0
  %arraydecay46 = getelementptr inbounds [33280 x { float, float }], [33280 x { float, float }]* %toBeEqualized, i64 0, i64 0
  %arraydecay47 = getelementptr inbounds [24960 x { float, float }], [24960 x { float, float }]* %equalized, i64 0, i64 0
  call void @do_recv_pipeline(i32 %conv14, float* nonnull %arraydecay15, i64 41800, float* nonnull %arraydecay27, i64 41800, i32* nonnull %recvd_msg_len_inner, i64 4, i8* nonnull %13, i64 1500, i8* nonnull %15, i64 18585, float* nonnull %ss_freq_offset, i64 4, i32* nonnull %num_sync_short_vals, i64 4, float* nonnull %sl_freq_offset, i64 4, i32* nonnull %num_sync_long_vals, i64 4, float* nonnull %arraydecay44, i64 133120, float* nonnull %arraydecay45, i64 133120, i32* nonnull %num_fft_outs, i64 4, { float, float }* nonnull %arraydecay46, i64 266240, { float, float }* nonnull %arraydecay47, i64 199680, i32* nonnull %num_eq_out_bits, i64 4, i32* nonnull %psdu, i64 4) #11
  %27 = load i32, i32* %recvd_msg_len_inner, align 4, !tbaa !2
  %call50 = call i32 @LZ4_decompress_safe(i8* nonnull %13, i8* nonnull %14, i32 %27, i32 2544) #11
  %cmp51 = icmp slt i32 %call50, 0
  br i1 %cmp51, label %if.then53, label %if.end55

if.then53:                                        ; preds = %if.end39
  %call54 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([32 x i8], [32 x i8]* @.str.29, i64 0, i64 0), i32 %call50)
  br label %if.end55

if.end55:                                         ; preds = %if.then53, %if.end39
  %28 = load i32, i32* @recv_count, align 4, !tbaa !2
  %29 = load i32, i32* @curr_obs, align 4, !tbaa !2
  %call56 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([55 x i8], [55 x i8]* @.str.30, i64 0, i64 0), i32 %28, i32 %29)
  %30 = bitcast %struct.Costmap2D_struct** %local_map to i8*
  call void @llvm.lifetime.start.p0i8(i64 8, i8* nonnull %30) #11
  %31 = load i32, i32* @curr_obs, align 4, !tbaa !2
  %idxprom = sext i32 %31 to i64
  %master_costmap = getelementptr inbounds %struct.Observation, %struct.Observation* %observations, i64 %idxprom, i32 6
  store %struct.Costmap2D_struct* %master_costmap, %struct.Costmap2D_struct** %local_map, align 8, !tbaa !6
  call void @fuseIntoLocal(%struct.Costmap2D_struct* nonnull %master_costmap, %struct.Costmap2D_struct* nonnull %uncmp_data) #11
  %call62 = call i32 (i8*, i64, i8*, ...) @snprintf(i8* nonnull %2, i64 9, i8* getelementptr inbounds ([7 x i8], [7 x i8]* @.str.31, i64 0, i64 0), i32 2544) #11
  %32 = load i32, i32* @car_sock, align 4, !tbaa !2
  %call64 = call i64 @send(i32 %32, i8* nonnull %2, i64 8, i32 0) #11
  %33 = load i32, i32* @car_sock, align 4, !tbaa !2
  %call66 = call i64 @send(i32 %33, i8* nonnull %30, i64 2544, i32 0) #11
  %34 = load i32, i32* @car_send_count, align 4, !tbaa !2
  %inc = add i32 %34, 1
  store i32 %inc, i32* @car_send_count, align 4, !tbaa !2
  %35 = load i32, i32* @recv_count, align 4, !tbaa !2
  %inc67 = add i32 %35, 1
  store i32 %inc67, i32* @recv_count, align 4, !tbaa !2
  call void @llvm.lifetime.end.p0i8(i64 8, i8* nonnull %30) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %26) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %25) #11
  call void @llvm.lifetime.end.p0i8(i64 199680, i8* nonnull %24) #11
  call void @llvm.lifetime.end.p0i8(i64 266240, i8* nonnull %23) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %22) #11
  call void @llvm.lifetime.end.p0i8(i64 133120, i8* nonnull %21) #11
  call void @llvm.lifetime.end.p0i8(i64 133120, i8* nonnull %20) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %19) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %18) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %17) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %16) #11
  call void @llvm.lifetime.end.p0i8(i64 18585, i8* nonnull %15) #11
  call void @llvm.lifetime.end.p0i8(i64 2544, i8* nonnull %14) #11
  call void @llvm.lifetime.end.p0i8(i64 1500, i8* nonnull %13) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %12) #11
  call void @llvm.lifetime.end.p0i8(i64 8, i8* nonnull %8) #11
  br label %if.end69

if.end69:                                         ; preds = %entry, %if.end55
  call void @llvm.lifetime.end.p0i8(i64 10, i8* nonnull %2) #11
  call void @llvm.lifetime.end.p0i8(i64 167200, i8* nonnull %1) #11
  call void @llvm.lifetime.end.p0i8(i64 167200, i8* nonnull %0) #11
  ret i8* null
}

declare dso_local i64 @send(i32, i8*, i64, i32) local_unnamed_addr #3

; Function Attrs: nofree nounwind
declare dso_local i64 @strtol(i8* readonly, i8** nocapture, i32) local_unnamed_addr #1

declare dso_local void @do_recv_pipeline(i32, float*, i64, float*, i64, i32*, i64, i8*, i64, i8*, i64, float*, i64, i32*, i64, float*, i64, i32*, i64, float*, i64, float*, i64, i32*, i64, { float, float }*, i64, { float, float }*, i64, i32*, i64, i32*, i64) local_unnamed_addr #3

declare dso_local i32 @LZ4_decompress_safe(i8*, i8*, i32, i32) local_unnamed_addr #3

declare dso_local void @fuseIntoLocal(%struct.Costmap2D_struct*, %struct.Costmap2D_struct*) local_unnamed_addr #3

; Function Attrs: nounwind uwtable
define dso_local noalias i8* @receive_and_fuse_maps(i8* nocapture readnone %parm_ptr, i64 %parm_ptr_sz) local_unnamed_addr #5 {
entry:
  %0 = load i32, i32* @lmap_count, align 4, !tbaa !2
  %cmp2 = icmp eq i32 %0, 0
  br i1 %cmp2, label %while.body, label %while.end

while.body:                                       ; preds = %entry, %while.body
  %call = tail call i32 @usleep(i32 1) #11
  %1 = load i32, i32* @lmap_count, align 4, !tbaa !2
  %cmp = icmp eq i32 %1, 0
  br i1 %cmp, label %while.body, label %while.end

while.end:                                        ; preds = %while.body, %entry
  %call1 = tail call i8* @receive_and_fuse_maps_impl(%struct.Observation* getelementptr inbounds ([2 x %struct.Observation], [2 x %struct.Observation]* @observationsArr, i64 0, i64 0), i64 undef)
  ret i8* null
}

declare dso_local i32 @usleep(i32) local_unnamed_addr #3

; Function Attrs: nounwind uwtable
define dso_local void @process_lidar_to_occgrid(%struct.lidar_inputs_struct* %lidar_inputs, i64 %lidarin_sz, %struct.Observation* %observations, i64 %observations_sz, i32* nocapture %n_cmp_bytes, i64 %n_cmp_bytes_sz, i8* %cmp_data, i64 %cmp_data_sz) local_unnamed_addr #5 {
entry:
  %0 = load i32, i32* @next_obs, align 4, !tbaa !2
  %idxprom = sext i32 %0 to i64
  %arrayidx = getelementptr inbounds %struct.Observation, %struct.Observation* %observations, i64 %idxprom
  %arraydecay = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 2, i64 0
  %1 = bitcast i8* %arraydecay to float*
  %data_size = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 1
  %2 = load i32, i32* %data_size, align 4, !tbaa !11
  %conv1 = ashr i32 %2, 2
  %arrayidx2 = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 0, i64 0
  %3 = load float, float* %arrayidx2, align 4, !tbaa !9
  %conv3 = fpext float %3 to double
  %arrayidx5 = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 0, i64 1
  %4 = load float, float* %arrayidx5, align 4, !tbaa !9
  %conv6 = fpext float %4 to double
  %arrayidx8 = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 0, i64 2
  %5 = load float, float* %arrayidx8, align 4, !tbaa !9
  %conv9 = fpext float %5 to double
  tail call void @cloudToOccgrid(%struct.Observation* %arrayidx, i64 %observations_sz, float* nonnull %1, i32 %conv1, double %conv3, double %conv6, double %conv9, double 1.500000e+00, i1 zeroext false, double 5.000000e-02, double 2.050000e+00, double 1.000000e+02, i32 100, i32 100, double 2.000000e+00) #11
  %6 = load i32, i32* @next_obs, align 4, !tbaa !2
  %idxprom10 = sext i32 %6 to i64
  %master_costmap = getelementptr inbounds %struct.Observation, %struct.Observation* %observations, i64 %idxprom10, i32 6
  %7 = load i32, i32* @curr_obs, align 4, !tbaa !2
  %sub = sub nsw i32 1, %7
  store i32 %sub, i32* @curr_obs, align 4, !tbaa !2
  %sub12 = sub nsw i32 1, %6
  store i32 %sub12, i32* @next_obs, align 4, !tbaa !2
  %8 = load i32, i32* @lmap_count, align 4, !tbaa !2
  %inc = add i32 %8, 1
  store i32 %inc, i32* @lmap_count, align 4, !tbaa !2
  %9 = bitcast %struct.Costmap2D_struct* %master_costmap to i8*
  %call = tail call i32 @LZ4_compress_default(i8* nonnull %9, i8* %cmp_data, i32 2544, i32 2544) #11
  store i32 %call, i32* %n_cmp_bytes, align 4, !tbaa !2
  ret void
}

declare dso_local void @cloudToOccgrid(%struct.Observation*, i64, float*, i32, double, double, double, double, i1 zeroext, double, double, double, i32, i32, double) local_unnamed_addr #3

declare dso_local i32 @LZ4_compress_default(i8*, i8*, i32, i32) local_unnamed_addr #3

; Function Attrs: nounwind uwtable
define dso_local void @encode_transmit_occgrid(i32* nocapture readonly %n_cmp_bytes, i64 %n_cmp_bytes_sz, i8* %cmp_data, i64 %cmp_data_sz) local_unnamed_addr #5 {
entry:
  %n_xmit_out = alloca i32, align 4
  %xmit_out_real = alloca [41800 x float], align 16
  %xmit_out_imag = alloca [41800 x float], align 16
  %psdu_len = alloca i32, align 4
  %pckt_hdr_out = alloca [64 x i8], align 16
  %pckt_hdr_len = alloca i32, align 4
  %msg_stream_real = alloca [24600 x float], align 16
  %msg_stream_imag = alloca [24600 x float], align 16
  %ofdm_car_str_real = alloca [33280 x float], align 16
  %ofdm_car_str_imag = alloca [33280 x float], align 16
  %ofc_res = alloca i32, align 4
  %fft_out_real = alloca [33280 x float], align 16
  %fft_out_imag = alloca [33280 x float], align 16
  %cycpref_out_real = alloca [41360 x float], align 16
  %cycpref_out_imag = alloca [41360 x float], align 16
  %w_buffer = alloca [10 x i8], align 1
  %0 = bitcast i32* %n_xmit_out to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %0) #11
  %1 = bitcast [41800 x float]* %xmit_out_real to i8*
  call void @llvm.lifetime.start.p0i8(i64 167200, i8* nonnull %1) #11
  %2 = bitcast [41800 x float]* %xmit_out_imag to i8*
  call void @llvm.lifetime.start.p0i8(i64 167200, i8* nonnull %2) #11
  %3 = bitcast i32* %psdu_len to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %3) #11
  store i32 0, i32* %psdu_len, align 4, !tbaa !2
  %4 = getelementptr inbounds [64 x i8], [64 x i8]* %pckt_hdr_out, i64 0, i64 0
  call void @llvm.lifetime.start.p0i8(i64 64, i8* nonnull %4) #11
  %5 = bitcast i32* %pckt_hdr_len to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %5) #11
  store i32 0, i32* %pckt_hdr_len, align 4, !tbaa !2
  %6 = bitcast [24600 x float]* %msg_stream_real to i8*
  call void @llvm.lifetime.start.p0i8(i64 98400, i8* nonnull %6) #11
  %7 = bitcast [24600 x float]* %msg_stream_imag to i8*
  call void @llvm.lifetime.start.p0i8(i64 98400, i8* nonnull %7) #11
  %8 = bitcast [33280 x float]* %ofdm_car_str_real to i8*
  call void @llvm.lifetime.start.p0i8(i64 133120, i8* nonnull %8) #11
  %9 = bitcast [33280 x float]* %ofdm_car_str_imag to i8*
  call void @llvm.lifetime.start.p0i8(i64 133120, i8* nonnull %9) #11
  %10 = bitcast i32* %ofc_res to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %10) #11
  store i32 0, i32* %ofc_res, align 4, !tbaa !2
  %11 = bitcast [33280 x float]* %fft_out_real to i8*
  call void @llvm.lifetime.start.p0i8(i64 133120, i8* nonnull %11) #11
  %12 = bitcast [33280 x float]* %fft_out_imag to i8*
  call void @llvm.lifetime.start.p0i8(i64 133120, i8* nonnull %12) #11
  %13 = bitcast [41360 x float]* %cycpref_out_real to i8*
  call void @llvm.lifetime.start.p0i8(i64 165440, i8* nonnull %13) #11
  %14 = bitcast [41360 x float]* %cycpref_out_imag to i8*
  call void @llvm.lifetime.start.p0i8(i64 165440, i8* nonnull %14) #11
  %15 = load i32, i32* %n_cmp_bytes, align 4, !tbaa !2
  %arraydecay = getelementptr inbounds [41800 x float], [41800 x float]* %xmit_out_real, i64 0, i64 0
  %arraydecay1 = getelementptr inbounds [41800 x float], [41800 x float]* %xmit_out_imag, i64 0, i64 0
  %arraydecay3 = getelementptr inbounds [24600 x float], [24600 x float]* %msg_stream_real, i64 0, i64 0
  %arraydecay4 = getelementptr inbounds [24600 x float], [24600 x float]* %msg_stream_imag, i64 0, i64 0
  %arraydecay5 = getelementptr inbounds [33280 x float], [33280 x float]* %ofdm_car_str_real, i64 0, i64 0
  %arraydecay6 = getelementptr inbounds [33280 x float], [33280 x float]* %ofdm_car_str_imag, i64 0, i64 0
  %arraydecay7 = getelementptr inbounds [33280 x float], [33280 x float]* %fft_out_real, i64 0, i64 0
  %arraydecay8 = getelementptr inbounds [33280 x float], [33280 x float]* %fft_out_imag, i64 0, i64 0
  %arraydecay9 = getelementptr inbounds [41360 x float], [41360 x float]* %cycpref_out_real, i64 0, i64 0
  %arraydecay10 = getelementptr inbounds [41360 x float], [41360 x float]* %cycpref_out_imag, i64 0, i64 0
  call void @do_xmit_pipeline(i32 %15, i8* %cmp_data, i64 %cmp_data_sz, i32* nonnull %n_xmit_out, i64 4, float* nonnull %arraydecay, i64 167200, float* nonnull %arraydecay1, i64 167200, i32* nonnull %psdu_len, i64 4, i8* nonnull %4, i64 64, i32* nonnull %pckt_hdr_len, i64 4, float* nonnull %arraydecay3, i64 98400, float* nonnull %arraydecay4, i64 98400, float* nonnull %arraydecay5, i64 133120, float* nonnull %arraydecay6, i64 133120, i32* nonnull %ofc_res, i64 4, float* nonnull %arraydecay7, i64 133120, float* nonnull %arraydecay8, i64 133120, float* nonnull %arraydecay9, i64 165440, float* nonnull %arraydecay10, i64 165440) #11
  %16 = getelementptr inbounds [10 x i8], [10 x i8]* %w_buffer, i64 0, i64 0
  call void @llvm.lifetime.start.p0i8(i64 10, i8* nonnull %16) #11
  %17 = load i32, i32* %n_xmit_out, align 4, !tbaa !2
  %mul = shl i32 %17, 2
  %call = call i32 (i8*, i64, i8*, ...) @snprintf(i8* nonnull %16, i64 9, i8* getelementptr inbounds ([7 x i8], [7 x i8]* @.str.31, i64 0, i64 0), i32 %mul) #11
  %18 = load i32, i32* @xmit_sock, align 4, !tbaa !2
  %call14 = call i64 @send(i32 %18, i8* nonnull %16, i64 8, i32 0) #11
  %19 = load i32, i32* @xmit_sock, align 4, !tbaa !2
  %20 = load i32, i32* %n_xmit_out, align 4, !tbaa !2
  %conv16 = sext i32 %20 to i64
  %mul17 = shl nsw i64 %conv16, 2
  %call18 = call i64 @send(i32 %19, i8* nonnull %1, i64 %mul17, i32 0) #11
  %21 = load i32, i32* @xmit_sock, align 4, !tbaa !2
  %22 = load i32, i32* %n_xmit_out, align 4, !tbaa !2
  %conv20 = sext i32 %22 to i64
  %mul21 = shl nsw i64 %conv20, 2
  %call22 = call i64 @send(i32 %21, i8* nonnull %2, i64 %mul21, i32 0) #11
  %23 = load i32, i32* @xmit_count, align 4, !tbaa !2
  %inc = add i32 %23, 1
  store i32 %inc, i32* @xmit_count, align 4, !tbaa !2
  call void @llvm.lifetime.end.p0i8(i64 10, i8* nonnull %16) #11
  call void @llvm.lifetime.end.p0i8(i64 165440, i8* nonnull %14) #11
  call void @llvm.lifetime.end.p0i8(i64 165440, i8* nonnull %13) #11
  call void @llvm.lifetime.end.p0i8(i64 133120, i8* nonnull %12) #11
  call void @llvm.lifetime.end.p0i8(i64 133120, i8* nonnull %11) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %10) #11
  call void @llvm.lifetime.end.p0i8(i64 133120, i8* nonnull %9) #11
  call void @llvm.lifetime.end.p0i8(i64 133120, i8* nonnull %8) #11
  call void @llvm.lifetime.end.p0i8(i64 98400, i8* nonnull %7) #11
  call void @llvm.lifetime.end.p0i8(i64 98400, i8* nonnull %6) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %5) #11
  call void @llvm.lifetime.end.p0i8(i64 64, i8* nonnull %4) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %3) #11
  call void @llvm.lifetime.end.p0i8(i64 167200, i8* nonnull %2) #11
  call void @llvm.lifetime.end.p0i8(i64 167200, i8* nonnull %1) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %0) #11
  ret void
}

declare dso_local void @do_xmit_pipeline(i32, i8*, i64, i32*, i64, float*, i64, float*, i64, i32*, i64, i8*, i64, i32*, i64, float*, i64, float*, i64, float*, i64, float*, i64, i32*, i64, float*, i64, float*, i64, float*, i64, float*, i64) local_unnamed_addr #3

; Function Attrs: nounwind uwtable
define dso_local noalias i8* @lidar_root(%struct.lidar_inputs_struct* %lidar_inputs, i64 %lidarin_sz, %struct.Observation* %observations, i64 %observations_sz, i32* nocapture %n_cmp_bytes, i64 %n_cmp_bytes_sz, i8* %cmp_data, i64 %cmp_data_sz) local_unnamed_addr #5 {
entry:
  tail call void @process_lidar_to_occgrid(%struct.lidar_inputs_struct* %lidar_inputs, i64 undef, %struct.Observation* %observations, i64 %observations_sz, i32* %n_cmp_bytes, i64 undef, i8* %cmp_data, i64 undef)
  ret i8* null
}

; Function Attrs: nounwind uwtable
define dso_local noalias i8* @cv_root(i32 %tr_val, i32* nocapture %out_label, i64 %outlabel_sz) local_unnamed_addr #5 {
entry:
  %call = tail call i32 @run_object_classification(i32 %tr_val) #11
  store i32 %call, i32* %out_label, align 4, !tbaa !8
  ret i8* null
}

declare dso_local i32 @run_object_classification(i32) local_unnamed_addr #3

; Function Attrs: nounwind uwtable
define dso_local i32 @main(i32 %argc, i8** %argv) local_unnamed_addr #5 {
entry:
  %bag_servaddr = alloca %struct.sockaddr_in, align 4
  %xmit_servaddr = alloca %struct.sockaddr_in, align 4
  %recv_servaddr = alloca %struct.sockaddr_in, align 4
  %car_servaddr = alloca %struct.sockaddr_in, align 4
  %l_buffer = alloca [20 x i8], align 16
  %lidar_inputs = alloca %struct.lidar_inputs_struct, align 4
  %ptr = alloca i8*, align 8
  %n_cmp_bytes = alloca i32, align 4
  %cmp_data = alloca [2544 x i8], align 16
  %ptr256 = alloca i8*, align 8
  %out_label = alloca i32, align 4
  %0 = bitcast %struct.sockaddr_in* %bag_servaddr to i8*
  call void @llvm.lifetime.start.p0i8(i64 16, i8* nonnull %0) #11
  %1 = bitcast %struct.sockaddr_in* %xmit_servaddr to i8*
  call void @llvm.lifetime.start.p0i8(i64 16, i8* nonnull %1) #11
  %2 = bitcast %struct.sockaddr_in* %recv_servaddr to i8*
  call void @llvm.lifetime.start.p0i8(i64 16, i8* nonnull %2) #11
  %3 = bitcast %struct.sockaddr_in* %car_servaddr to i8*
  call void @llvm.lifetime.start.p0i8(i64 16, i8* nonnull %3) #11
  %4 = getelementptr inbounds [20 x i8], [20 x i8]* %l_buffer, i64 0, i64 0
  call void @llvm.lifetime.start.p0i8(i64 20, i8* nonnull %4) #11
  call void @llvm.memset.p0i8.i64(i8* nonnull align 16 %4, i8 0, i64 20, i1 false)
  %5 = bitcast %struct.lidar_inputs_struct* %lidar_inputs to i8*
  call void @llvm.lifetime.start.p0i8(i64 200020, i8* nonnull %5) #11
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 16 getelementptr inbounds ([20 x i8], [20 x i8]* @bag_inet_addr_str, i64 0, i64 0), i8* align 1 getelementptr inbounds ([10 x i8], [10 x i8]* @.str.32, i64 0, i64 0), i64 10, i1 false)
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 16 getelementptr inbounds ([20 x i8], [20 x i8]* @wifi_inet_addr_str, i64 0, i64 0), i8* align 1 getelementptr inbounds ([10 x i8], [10 x i8]* @.str.32, i64 0, i64 0), i64 10, i1 false)
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 16 getelementptr inbounds ([20 x i8], [20 x i8]* @car_inet_addr_str, i64 0, i64 0), i8* align 1 getelementptr inbounds ([10 x i8], [10 x i8]* @.str.32, i64 0, i64 0), i64 10, i1 false)
  %puts = tail call i32 @puts(i8* getelementptr inbounds ([34 x i8], [34 x i8]* @str.104, i64 0, i64 0))
  tail call void @init_occgrid_state() #11
  %puts347 = tail call i32 @puts(i8* getelementptr inbounds ([38 x i8], [38 x i8]* @str.105, i64 0, i64 0))
  tail call void (...) @xmit_pipe_init() #11
  %puts348 = tail call i32 @puts(i8* getelementptr inbounds ([37 x i8], [37 x i8]* @str.106, i64 0, i64 0))
  tail call void (...) @recv_pipe_init() #11
  %puts349 = tail call i32 @puts(i8* getelementptr inbounds ([44 x i8], [44 x i8]* @str.107, i64 0, i64 0))
  %call7 = tail call i32 (...) @cv_toolset_init() #11
  %cmp = icmp eq i32 %call7, 1
  br i1 %cmp, label %if.end, label %if.then

if.then:                                          ; preds = %entry
  %puts370 = tail call i32 @puts(i8* getelementptr inbounds ([49 x i8], [49 x i8]* @str.128, i64 0, i64 0))
  tail call void @exit(i32 0) #12
  unreachable

if.end:                                           ; preds = %entry
  %call9 = tail call void (i32)* @signal(i32 2, void (i32)* nonnull @INThandler) #11
  %call10 = tail call void (i32)* @signal(i32 13, void (i32)* nonnull @SIGPIPE_handler) #11
  br label %while.cond

while.cond:                                       ; preds = %while.cond.backedge, %if.end
  %call11 = tail call i32 @getopt(i32 %argc, i8** %argv, i8* getelementptr inbounds ([11 x i8], [11 x i8]* @.str.38, i64 0, i64 0)) #11
  switch i32 %call11, label %while.cond.backedge [
    i32 -1, label %while.end
    i32 104, label %sw.bb
    i32 66, label %sw.bb13
    i32 87, label %sw.bb15
    i32 67, label %sw.bb17
    i32 115, label %sw.bb19
    i32 58, label %sw.bb21
    i32 63, label %sw.bb23
  ]

while.cond.backedge:                              ; preds = %while.cond, %sw.bb23, %sw.bb21, %sw.bb19, %sw.bb17, %sw.bb15, %sw.bb13
  br label %while.cond

sw.bb:                                            ; preds = %while.cond
  %6 = load i8*, i8** %argv, align 8, !tbaa !6
  tail call void @print_usage(i8* %6)
  tail call void @exit(i32 0) #12
  unreachable

sw.bb13:                                          ; preds = %while.cond
  %7 = load i8*, i8** @optarg, align 8, !tbaa !6
  %call14 = tail call i32 (i8*, i64, i8*, ...) @snprintf(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @bag_inet_addr_str, i64 0, i64 0), i64 20, i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str.39, i64 0, i64 0), i8* %7) #11
  br label %while.cond.backedge

sw.bb15:                                          ; preds = %while.cond
  %8 = load i8*, i8** @optarg, align 8, !tbaa !6
  %call16 = tail call i32 (i8*, i64, i8*, ...) @snprintf(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @wifi_inet_addr_str, i64 0, i64 0), i64 20, i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str.39, i64 0, i64 0), i8* %8) #11
  br label %while.cond.backedge

sw.bb17:                                          ; preds = %while.cond
  %9 = load i8*, i8** @optarg, align 8, !tbaa !6
  %call18 = tail call i32 (i8*, i64, i8*, ...) @snprintf(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @car_inet_addr_str, i64 0, i64 0), i64 20, i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str.39, i64 0, i64 0), i8* %9) #11
  br label %while.cond.backedge

sw.bb19:                                          ; preds = %while.cond
  %10 = load i8*, i8** @optarg, align 8, !tbaa !6
  %call20 = tail call i32 @atoi(i8* %10) #13
  store i32 %call20, i32* @max_time_steps, align 4, !tbaa !2
  br label %while.cond.backedge

sw.bb21:                                          ; preds = %while.cond
  %puts369 = tail call i32 @puts(i8* getelementptr inbounds ([21 x i8], [21 x i8]* @str.127, i64 0, i64 0))
  br label %while.cond.backedge

sw.bb23:                                          ; preds = %while.cond
  %11 = load i32, i32* @optopt, align 4, !tbaa !2
  %call24 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @.str.41, i64 0, i64 0), i32 %11)
  br label %while.cond.backedge

while.end:                                        ; preds = %while.cond
  %12 = load i32, i32* @max_time_steps, align 4, !tbaa !2
  %cmp25 = icmp eq i32 %12, -2
  br i1 %cmp25, label %if.else, label %if.then26

if.then26:                                        ; preds = %while.end
  %call27 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([31 x i8], [31 x i8]* @.str.42, i64 0, i64 0), i32 %12)
  br label %if.end29

if.else:                                          ; preds = %while.end
  %puts350 = tail call i32 @puts(i8* getelementptr inbounds ([29 x i8], [29 x i8]* @str.108, i64 0, i64 0))
  br label %if.end29

if.end29:                                         ; preds = %if.else, %if.then26
  %call30 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([43 x i8], [43 x i8]* @.str.44, i64 0, i64 0), i8* getelementptr inbounds ([20 x i8], [20 x i8]* @bag_inet_addr_str, i64 0, i64 0), i32 5556)
  %call31 = tail call i32 @socket(i32 2, i32 1, i32 0) #11
  store i32 %call31, i32* @bag_sock, align 4, !tbaa !2
  %cmp32 = icmp slt i32 %call31, 0
  br i1 %cmp32, label %if.then33, label %if.else35

if.then33:                                        ; preds = %if.end29
  %puts368 = tail call i32 @puts(i8* getelementptr inbounds ([30 x i8], [30 x i8]* @str.126, i64 0, i64 0))
  tail call void @exit(i32 0) #12
  unreachable

if.else35:                                        ; preds = %if.end29
  %puts351 = tail call i32 @puts(i8* getelementptr inbounds ([34 x i8], [34 x i8]* @str.109, i64 0, i64 0))
  %sin_family = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %bag_servaddr, i64 0, i32 0
  store i16 2, i16* %sin_family, align 4, !tbaa !13
  %call38 = tail call i32 @inet_addr(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @bag_inet_addr_str, i64 0, i64 0)) #11
  %s_addr = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %bag_servaddr, i64 0, i32 2, i32 0
  store i32 %call38, i32* %s_addr, align 4, !tbaa !17
  %sin_port = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %bag_servaddr, i64 0, i32 1
  store i16 -19435, i16* %sin_port, align 2, !tbaa !18
  %13 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %14 = bitcast %struct.sockaddr_in* %bag_servaddr to %struct.sockaddr*
  %call47386 = call i32 @connect(i32 %13, %struct.sockaddr* nonnull %14, i32 16) #11
  %cmp48387 = icmp eq i32 %call47386, 0
  br i1 %cmp48387, label %if.else53, label %if.then50

if.then50:                                        ; preds = %if.else35, %if.then50
  %puts367 = call i32 @puts(i8* getelementptr inbounds ([41 x i8], [41 x i8]* @str.125, i64 0, i64 0))
  %call52 = call i32 @sleep(i32 1) #11
  %15 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %call47 = call i32 @connect(i32 %15, %struct.sockaddr* nonnull %14, i32 16) #11
  %cmp48 = icmp eq i32 %call47, 0
  br i1 %cmp48, label %if.else53, label %if.then50

if.else53:                                        ; preds = %if.then50, %if.else35
  %puts352 = call i32 @puts(i8* getelementptr inbounds ([30 x i8], [30 x i8]* @str.110, i64 0, i64 0))
  %call56 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([44 x i8], [44 x i8]* @.str.49, i64 0, i64 0), i8* getelementptr inbounds ([20 x i8], [20 x i8]* @wifi_inet_addr_str, i64 0, i64 0), i32 5558)
  %call57 = call i32 @socket(i32 2, i32 1, i32 0) #11
  store i32 %call57, i32* @xmit_sock, align 4, !tbaa !2
  %cmp58 = icmp slt i32 %call57, 0
  br i1 %cmp58, label %if.then60, label %if.else62

if.then60:                                        ; preds = %if.else53
  %puts366 = call i32 @puts(i8* getelementptr inbounds ([36 x i8], [36 x i8]* @str.124, i64 0, i64 0))
  call void @exit(i32 0) #12
  unreachable

if.else62:                                        ; preds = %if.else53
  %puts353 = call i32 @puts(i8* getelementptr inbounds ([40 x i8], [40 x i8]* @str.111, i64 0, i64 0))
  %sin_family65 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %xmit_servaddr, i64 0, i32 0
  store i16 2, i16* %sin_family65, align 4, !tbaa !13
  %call66 = call i32 @inet_addr(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @wifi_inet_addr_str, i64 0, i64 0)) #11
  %s_addr68 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %xmit_servaddr, i64 0, i32 2, i32 0
  store i32 %call66, i32* %s_addr68, align 4, !tbaa !17
  %sin_port83 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %xmit_servaddr, i64 0, i32 1
  store i16 -18923, i16* %sin_port83, align 2, !tbaa !18
  %16 = load i32, i32* @xmit_sock, align 4, !tbaa !2
  %17 = bitcast %struct.sockaddr_in* %xmit_servaddr to %struct.sockaddr*
  %call86384 = call i32 @connect(i32 %16, %struct.sockaddr* nonnull %17, i32 16) #11
  %cmp87385 = icmp eq i32 %call86384, 0
  br i1 %cmp87385, label %if.else92, label %if.then89

if.then89:                                        ; preds = %if.else62, %if.then89
  %puts365 = call i32 @puts(i8* getelementptr inbounds ([47 x i8], [47 x i8]* @str.123, i64 0, i64 0))
  %call91 = call i32 @sleep(i32 1) #11
  %18 = load i32, i32* @xmit_sock, align 4, !tbaa !2
  %call86 = call i32 @connect(i32 %18, %struct.sockaddr* nonnull %17, i32 16) #11
  %cmp87 = icmp eq i32 %call86, 0
  br i1 %cmp87, label %if.else92, label %if.then89

if.else92:                                        ; preds = %if.then89, %if.else62
  %puts354 = call i32 @puts(i8* getelementptr inbounds ([36 x i8], [36 x i8]* @str.112, i64 0, i64 0))
  %call95 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([44 x i8], [44 x i8]* @.str.54, i64 0, i64 0), i8* getelementptr inbounds ([20 x i8], [20 x i8]* @wifi_inet_addr_str, i64 0, i64 0), i32 5560)
  %call96 = call i32 @socket(i32 2, i32 1, i32 0) #11
  store i32 %call96, i32* @recv_sock, align 4, !tbaa !2
  %cmp97 = icmp slt i32 %call96, 0
  br i1 %cmp97, label %if.then99, label %if.else101

if.then99:                                        ; preds = %if.else92
  %puts364 = call i32 @puts(i8* getelementptr inbounds ([36 x i8], [36 x i8]* @str.122, i64 0, i64 0))
  call void @exit(i32 0) #12
  unreachable

if.else101:                                       ; preds = %if.else92
  %puts355 = call i32 @puts(i8* getelementptr inbounds ([40 x i8], [40 x i8]* @str.113, i64 0, i64 0))
  %sin_family104 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %recv_servaddr, i64 0, i32 0
  store i16 2, i16* %sin_family104, align 4, !tbaa !13
  %call105 = call i32 @inet_addr(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @wifi_inet_addr_str, i64 0, i64 0)) #11
  %s_addr107 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %recv_servaddr, i64 0, i32 2, i32 0
  store i32 %call105, i32* %s_addr107, align 4, !tbaa !17
  %sin_port122 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %recv_servaddr, i64 0, i32 1
  store i16 -18411, i16* %sin_port122, align 2, !tbaa !18
  %19 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %20 = bitcast %struct.sockaddr_in* %recv_servaddr to %struct.sockaddr*
  %call125382 = call i32 @connect(i32 %19, %struct.sockaddr* nonnull %20, i32 16) #11
  %cmp126383 = icmp eq i32 %call125382, 0
  br i1 %cmp126383, label %if.else131, label %if.then128

if.then128:                                       ; preds = %if.else101, %if.then128
  %puts363 = call i32 @puts(i8* getelementptr inbounds ([47 x i8], [47 x i8]* @str.121, i64 0, i64 0))
  %call130 = call i32 @sleep(i32 1) #11
  %21 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %call125 = call i32 @connect(i32 %21, %struct.sockaddr* nonnull %20, i32 16) #11
  %cmp126 = icmp eq i32 %call125, 0
  br i1 %cmp126, label %if.else131, label %if.then128

if.else131:                                       ; preds = %if.then128, %if.else101
  %puts356 = call i32 @puts(i8* getelementptr inbounds ([36 x i8], [36 x i8]* @str.114, i64 0, i64 0))
  %call134 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([43 x i8], [43 x i8]* @.str.59, i64 0, i64 0), i8* getelementptr inbounds ([20 x i8], [20 x i8]* @car_inet_addr_str, i64 0, i64 0), i32 5562)
  %call135 = call i32 @socket(i32 2, i32 1, i32 0) #11
  store i32 %call135, i32* @car_sock, align 4, !tbaa !2
  %cmp136 = icmp slt i32 %call135, 0
  br i1 %cmp136, label %if.then138, label %if.else140

if.then138:                                       ; preds = %if.else131
  %puts362 = call i32 @puts(i8* getelementptr inbounds ([30 x i8], [30 x i8]* @str.120, i64 0, i64 0))
  call void @exit(i32 0) #12
  unreachable

if.else140:                                       ; preds = %if.else131
  %puts357 = call i32 @puts(i8* getelementptr inbounds ([34 x i8], [34 x i8]* @str.115, i64 0, i64 0))
  %sin_family143 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %car_servaddr, i64 0, i32 0
  store i16 2, i16* %sin_family143, align 4, !tbaa !13
  %call144 = call i32 @inet_addr(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @car_inet_addr_str, i64 0, i64 0)) #11
  %s_addr146 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %car_servaddr, i64 0, i32 2, i32 0
  store i32 %call144, i32* %s_addr146, align 4, !tbaa !17
  %sin_port161 = getelementptr inbounds %struct.sockaddr_in, %struct.sockaddr_in* %car_servaddr, i64 0, i32 1
  store i16 -17899, i16* %sin_port161, align 2, !tbaa !18
  %22 = load i32, i32* @car_sock, align 4, !tbaa !2
  %23 = bitcast %struct.sockaddr_in* %car_servaddr to %struct.sockaddr*
  %call164380 = call i32 @connect(i32 %22, %struct.sockaddr* nonnull %23, i32 16) #11
  %cmp165381 = icmp eq i32 %call164380, 0
  br i1 %cmp165381, label %if.else170, label %if.then167

if.then167:                                       ; preds = %if.else140, %if.then167
  %puts361 = call i32 @puts(i8* getelementptr inbounds ([41 x i8], [41 x i8]* @str.119, i64 0, i64 0))
  %call169 = call i32 @sleep(i32 1) #11
  %24 = load i32, i32* @car_sock, align 4, !tbaa !2
  %call164 = call i32 @connect(i32 %24, %struct.sockaddr* nonnull %23, i32 16) #11
  %cmp165 = icmp eq i32 %call164, 0
  br i1 %cmp165, label %if.else170, label %if.then167

if.else170:                                       ; preds = %if.then167, %if.else140
  %puts358 = call i32 @puts(i8* getelementptr inbounds ([30 x i8], [30 x i8]* @str.116, i64 0, i64 0))
  %call173 = call i32 @gettimeofday(%struct.timeval* nonnull @start_prog, %struct.timezone* null) #11
  %arrayidx251 = getelementptr inbounds [20 x i8], [20 x i8]* %l_buffer, i64 0, i64 9
  %25 = bitcast i8** %ptr256 to i8*
  %add.ptr259 = getelementptr inbounds [20 x i8], [20 x i8]* %l_buffer, i64 0, i64 1
  %26 = bitcast [20 x i8]* %l_buffer to i32*
  %add.ptr279 = getelementptr inbounds [20 x i8], [20 x i8]* %l_buffer, i64 0, i64 4
  %27 = bitcast i8* %add.ptr279 to i32*
  %add.ptr281 = getelementptr inbounds [20 x i8], [20 x i8]* %l_buffer, i64 0, i64 8
  %28 = bitcast i8* %add.ptr281 to float*
  %arrayidx197 = getelementptr inbounds [20 x i8], [20 x i8]* %l_buffer, i64 0, i64 9
  %29 = bitcast i8** %ptr to i8*
  %add.ptr = getelementptr inbounds [20 x i8], [20 x i8]* %l_buffer, i64 0, i64 1
  %arraydecay207 = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 2, i64 0
  %30 = bitcast %struct.lidar_inputs_struct* %lidar_inputs to i32*
  %arrayidx227 = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 0, i64 1
  %31 = bitcast float* %arrayidx227 to i32*
  %arrayidx229 = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 0, i64 2
  %32 = bitcast float* %arrayidx229 to i32*
  %data_size = getelementptr inbounds %struct.lidar_inputs_struct, %struct.lidar_inputs_struct* %lidar_inputs, i64 0, i32 1
  %33 = bitcast i32* %n_cmp_bytes to i8*
  %34 = getelementptr inbounds [2544 x i8], [2544 x i8]* %cmp_data, i64 0, i64 0
  %35 = bitcast i32* %out_label to i8*
  br label %land.rhs

land.rhs:                                         ; preds = %if.else170, %if.end289
  %hit_eof.0379 = phi i8 [ 0, %if.else170 ], [ %hit_eof.1, %if.end289 ]
  %36 = load i32, i32* @lidar_count, align 4, !tbaa !2
  %37 = load i32, i32* @max_time_steps, align 4, !tbaa !2
  %cmp175 = icmp ult i32 %36, %37
  br i1 %cmp175, label %while.body177, label %while.end303

while.body177:                                    ; preds = %land.rhs
  %call178 = call i32 @gettimeofday(%struct.timeval* nonnull @start_proc_rdbag, %struct.timezone* null) #11
  %38 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %call179 = call i32 @read_all(i32 %38, i8* nonnull %4, i32 10)
  %call180 = call i32 @gettimeofday(%struct.timeval* nonnull @stop_proc_rdbag, %struct.timezone* null) #11
  %39 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_proc_rdbag, i64 0, i32 0), align 8, !tbaa !19
  %40 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_rdbag, i64 0, i32 0), align 8, !tbaa !19
  %sub = sub i64 %39, %40
  %41 = load i64, i64* @proc_rdbag_sec, align 8, !tbaa !22
  %add = add i64 %sub, %41
  store i64 %add, i64* @proc_rdbag_sec, align 8, !tbaa !22
  %42 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_proc_rdbag, i64 0, i32 1), align 8, !tbaa !23
  %43 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_rdbag, i64 0, i32 1), align 8, !tbaa !23
  %sub181 = sub i64 %42, %43
  %44 = load i64, i64* @proc_rdbag_usec, align 8, !tbaa !22
  %add182 = add i64 %sub181, %44
  store i64 %add182, i64* @proc_rdbag_usec, align 8, !tbaa !22
  %cmp183 = icmp slt i32 %call179, 10
  br i1 %cmp183, label %if.then185, label %if.end192

if.then185:                                       ; preds = %while.body177
  %cmp186 = icmp eq i32 %call179, 0
  br i1 %cmp186, label %if.end192, label %if.else189

if.else189:                                       ; preds = %if.then185
  %call190 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([50 x i8], [50 x i8]* @.str.64, i64 0, i64 0), i32 %call179, i32 10)
  call void @closeout_and_exit(i8* getelementptr inbounds ([30 x i8], [30 x i8]* @.str.65, i64 0, i64 0), i32 -1)
  unreachable

if.end192:                                        ; preds = %if.then185, %while.body177
  %hit_eof.1 = phi i8 [ %hit_eof.0379, %while.body177 ], [ 1, %if.then185 ]
  %45 = load i8, i8* %4, align 16, !tbaa !8
  switch i8 %45, label %if.end289 [
    i8 76, label %land.lhs.true
    i8 79, label %land.lhs.true250
  ]

land.lhs.true:                                    ; preds = %if.end192
  %46 = load i8, i8* %arrayidx197, align 1, !tbaa !8
  %cmp199 = icmp eq i8 %46, 76
  br i1 %cmp199, label %if.then201, label %if.end289

if.then201:                                       ; preds = %land.lhs.true
  %call202 = call i32 @gettimeofday(%struct.timeval* nonnull @start_proc_lidar, %struct.timezone* null) #11
  call void @llvm.lifetime.start.p0i8(i64 8, i8* nonnull %29) #11
  %call204 = call i64 @strtol(i8* nonnull %add.ptr, i8** nonnull %ptr, i32 10) #11
  %conv205 = trunc i64 %call204 to i32
  %47 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %48 = load i8*, i8** @ack, align 8, !tbaa !6
  %call206 = call i64 @send(i32 %47, i8* %48, i64 2, i32 0) #11
  %49 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %call208 = call i32 @read_all(i32 %49, i8* nonnull %arraydecay207, i32 %conv205)
  %cmp209 = icmp slt i32 %call208, %conv205
  br i1 %cmp209, label %if.then211, label %if.end219

if.then211:                                       ; preds = %if.then201
  %cmp212 = icmp eq i32 %call208, 0
  br i1 %cmp212, label %if.then214, label %if.else216

if.then214:                                       ; preds = %if.then211
  %puts360 = call i32 @puts(i8* getelementptr inbounds ([48 x i8], [48 x i8]* @str.118, i64 0, i64 0))
  call void @closeout_and_exit(i8* getelementptr inbounds ([29 x i8], [29 x i8]* @.str.67, i64 0, i64 0), i32 -1)
  unreachable

if.else216:                                       ; preds = %if.then211
  %conv205.le = trunc i64 %call204 to i32
  %call217 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([65 x i8], [65 x i8]* @.str.68, i64 0, i64 0), i32 %call208, i32 %conv205.le)
  call void @closeout_and_exit(i8* getelementptr inbounds ([32 x i8], [32 x i8]* @.str.69, i64 0, i64 0), i32 -1)
  unreachable

if.end219:                                        ; preds = %if.then201
  %cmp220 = icmp sgt i32 %call208, %conv205
  br i1 %cmp220, label %if.then222, label %if.end224

if.then222:                                       ; preds = %if.end219
  %call223 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([53 x i8], [53 x i8]* @.str.70, i64 0, i64 0), i32 %call208, i32 %conv205)
  br label %if.end224

if.end224:                                        ; preds = %if.then222, %if.end219
  %50 = load i32, i32* bitcast ([3 x float]* @odometry to i32*), align 4, !tbaa !9
  store i32 %50, i32* %30, align 4, !tbaa !9
  %51 = load i32, i32* bitcast (float* getelementptr inbounds ([3 x float], [3 x float]* @odometry, i64 0, i64 1) to i32*), align 4, !tbaa !9
  store i32 %51, i32* %31, align 4, !tbaa !9
  %52 = load i32, i32* bitcast (float* getelementptr inbounds ([3 x float], [3 x float]* @odometry, i64 0, i64 2) to i32*), align 4, !tbaa !9
  store i32 %52, i32* %32, align 4, !tbaa !9
  store i32 %call208, i32* %data_size, align 4, !tbaa !11
  %53 = load i32, i32* @lidar_count, align 4, !tbaa !2
  %call230 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([31 x i8], [31 x i8]* @.str.71, i64 0, i64 0), i32 %53)
  %call231 = call i32 @gettimeofday(%struct.timeval* nonnull @start_proc_data, %struct.timezone* null) #11
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %33) #11
  store i32 0, i32* %n_cmp_bytes, align 4, !tbaa !2
  call void @llvm.lifetime.start.p0i8(i64 2544, i8* nonnull %34) #11
  %call233 = call i8* @lidar_root(%struct.lidar_inputs_struct* nonnull %lidar_inputs, i64 undef, %struct.Observation* getelementptr inbounds ([2 x %struct.Observation], [2 x %struct.Observation]* @observationsArr, i64 0, i64 0), i64 5216, i32* nonnull %n_cmp_bytes, i64 undef, i8* nonnull %34, i64 undef)
  call void @encode_transmit_occgrid(i32* nonnull %n_cmp_bytes, i64 undef, i8* nonnull %34, i64 2544)
  %call235 = call i8* @receive_and_fuse_maps(i8* undef, i64 undef)
  %54 = load i32, i32* @lidar_count, align 4, !tbaa !2
  %inc = add i32 %54, 1
  store i32 %inc, i32* @lidar_count, align 4, !tbaa !2
  %call236 = call i32 @gettimeofday(%struct.timeval* nonnull @stop_proc_lidar, %struct.timezone* null) #11
  %55 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_proc_lidar, i64 0, i32 0), align 8, !tbaa !19
  %56 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_data, i64 0, i32 0), align 8, !tbaa !19
  %sub237 = sub i64 %55, %56
  %57 = load i64, i64* @proc_data_sec, align 8, !tbaa !22
  %add238 = add i64 %sub237, %57
  store i64 %add238, i64* @proc_data_sec, align 8, !tbaa !22
  %58 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_proc_lidar, i64 0, i32 1), align 8, !tbaa !23
  %59 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_data, i64 0, i32 1), align 8, !tbaa !23
  %sub239 = sub i64 %58, %59
  %60 = load i64, i64* @proc_data_usec, align 8, !tbaa !22
  %add240 = add i64 %sub239, %60
  store i64 %add240, i64* @proc_data_usec, align 8, !tbaa !22
  %61 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_lidar, i64 0, i32 0), align 8, !tbaa !19
  %sub241 = sub i64 %55, %61
  %62 = load i64, i64* @proc_lidar_sec, align 8, !tbaa !22
  %add242 = add i64 %sub241, %62
  store i64 %add242, i64* @proc_lidar_sec, align 8, !tbaa !22
  %63 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_lidar, i64 0, i32 1), align 8, !tbaa !23
  %sub243 = sub i64 %58, %63
  %64 = load i64, i64* @proc_lidar_usec, align 8, !tbaa !22
  %add244 = add i64 %sub243, %64
  store i64 %add244, i64* @proc_lidar_usec, align 8, !tbaa !22
  call void @llvm.lifetime.end.p0i8(i64 2544, i8* nonnull %34) #11
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %33) #11
  call void @llvm.lifetime.end.p0i8(i64 8, i8* nonnull %29) #11
  br label %if.end289

land.lhs.true250:                                 ; preds = %if.end192
  %65 = load i8, i8* %arrayidx251, align 1, !tbaa !8
  %cmp253 = icmp eq i8 %65, 79
  br i1 %cmp253, label %if.then255, label %if.end289

if.then255:                                       ; preds = %land.lhs.true250
  call void @llvm.lifetime.start.p0i8(i64 8, i8* nonnull %25) #11
  %call260 = call i64 @strtol(i8* nonnull %add.ptr259, i8** nonnull %ptr256, i32 10) #11
  %conv261 = trunc i64 %call260 to i32
  %66 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %67 = load i8*, i8** @ack, align 8, !tbaa !6
  %call262 = call i64 @send(i32 %66, i8* %67, i64 2, i32 0) #11
  %68 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %call265 = call i32 @read_all(i32 %68, i8* nonnull %4, i32 %conv261)
  %cmp266 = icmp slt i32 %call265, %conv261
  br i1 %cmp266, label %if.then268, label %if.end276

if.then268:                                       ; preds = %if.then255
  %cmp269 = icmp eq i32 %call265, 0
  br i1 %cmp269, label %if.then271, label %if.else273

if.then271:                                       ; preds = %if.then268
  %puts359 = call i32 @puts(i8* getelementptr inbounds ([46 x i8], [46 x i8]* @str.117, i64 0, i64 0))
  call void @closeout_and_exit(i8* getelementptr inbounds ([32 x i8], [32 x i8]* @.str.73, i64 0, i64 0), i32 -1)
  unreachable

if.else273:                                       ; preds = %if.then268
  %conv261.le = trunc i64 %call260 to i32
  %call274 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([63 x i8], [63 x i8]* @.str.74, i64 0, i64 0), i32 %call265, i32 %conv261.le)
  call void @closeout_and_exit(i8* getelementptr inbounds ([35 x i8], [35 x i8]* @.str.75, i64 0, i64 0), i32 -1)
  unreachable

if.end276:                                        ; preds = %if.then255
  %69 = load i32, i32* %26, align 16, !tbaa !9
  store i32 %69, i32* bitcast ([3 x float]* @odometry to i32*), align 4, !tbaa !9
  %70 = load i32, i32* %27, align 4, !tbaa !9
  store i32 %70, i32* bitcast (float* getelementptr inbounds ([3 x float], [3 x float]* @odometry, i64 0, i64 1) to i32*), align 4, !tbaa !9
  %71 = load float, float* %28, align 8, !tbaa !9
  store float %71, float* getelementptr inbounds ([3 x float], [3 x float]* @odometry, i64 0, i64 2), align 4, !tbaa !9
  %72 = load i32, i32* @odo_count, align 4, !tbaa !2
  %.cast = bitcast i32 %69 to float
  %conv282 = fpext float %.cast to double
  %.cast388 = bitcast i32 %70 to float
  %conv283 = fpext float %.cast388 to double
  %conv284 = fpext float %71 to double
  %call285 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([34 x i8], [34 x i8]* @.str.76, i64 0, i64 0), i32 %72, double %conv282, double %conv283, double %conv284)
  %73 = load i32, i32* @odo_count, align 4, !tbaa !2
  %inc286 = add i32 %73, 1
  store i32 %inc286, i32* @odo_count, align 4, !tbaa !2
  call void @llvm.lifetime.end.p0i8(i64 8, i8* nonnull %25) #11
  br label %if.end289

if.end289:                                        ; preds = %if.end192, %land.lhs.true, %if.end276, %land.lhs.true250, %if.end224
  %call290 = call i32 @gettimeofday(%struct.timeval* nonnull @start_proc_cv, %struct.timezone* null) #11
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %35) #11
  %call291 = call i8* @cv_root(i32 1, i32* nonnull %out_label, i64 undef)
  %74 = load i32, i32* @cv_count, align 4, !tbaa !2
  %inc292 = add i32 %74, 1
  store i32 %inc292, i32* @cv_count, align 4, !tbaa !2
  %call293 = call i32 @gettimeofday(%struct.timeval* nonnull @stop_proc_cv, %struct.timezone* null) #11
  %75 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_proc_cv, i64 0, i32 0), align 8, !tbaa !19
  %76 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_cv, i64 0, i32 0), align 8, !tbaa !19
  %sub294 = sub nsw i64 %75, %76
  %77 = load i64, i64* @proc_cv_sec, align 8, !tbaa !22
  %add295 = add i64 %77, %sub294
  store i64 %add295, i64* @proc_cv_sec, align 8, !tbaa !22
  %78 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_proc_cv, i64 0, i32 1), align 8, !tbaa !23
  %79 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_proc_cv, i64 0, i32 1), align 8, !tbaa !23
  %sub296 = sub nsw i64 %78, %79
  %80 = load i64, i64* @proc_cv_usec, align 8, !tbaa !22
  %add297 = add i64 %80, %sub296
  store i64 %add297, i64* @proc_cv_usec, align 8, !tbaa !22
  %mul = mul nsw i64 %sub294, 1000000
  %add300 = add nsw i64 %sub296, %mul
  %call301 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([44 x i8], [44 x i8]* @.str.77, i64 0, i64 0), i64 %add300)
  %81 = load i32, i32* %out_label, align 4, !tbaa !8
  %call302 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([39 x i8], [39 x i8]* @.str.78, i64 0, i64 0), i32 %81)
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %35) #11
  %82 = and i8 %hit_eof.1, 1
  %tobool = icmp eq i8 %82, 0
  br i1 %tobool, label %land.rhs, label %while.end303

while.end303:                                     ; preds = %if.end289, %land.rhs
  call void @dump_final_run_statistics()
  %83 = load i32, i32* @bag_sock, align 4, !tbaa !2
  %call304 = call i32 @close(i32 %83) #11
  %84 = load i32, i32* @xmit_sock, align 4, !tbaa !2
  %call305 = call i32 @close(i32 %84) #11
  %85 = load i32, i32* @recv_sock, align 4, !tbaa !2
  %call306 = call i32 @close(i32 %85) #11
  call void @llvm.lifetime.end.p0i8(i64 200020, i8* nonnull %5) #11
  call void @llvm.lifetime.end.p0i8(i64 20, i8* nonnull %4) #11
  call void @llvm.lifetime.end.p0i8(i64 16, i8* nonnull %3) #11
  call void @llvm.lifetime.end.p0i8(i64 16, i8* nonnull %2) #11
  call void @llvm.lifetime.end.p0i8(i64 16, i8* nonnull %1) #11
  call void @llvm.lifetime.end.p0i8(i64 16, i8* nonnull %0) #11
  ret i32 0
}

; Function Attrs: argmemonly nounwind
declare void @llvm.memset.p0i8.i64(i8* nocapture writeonly, i8, i64, i1 immarg) #6

declare dso_local void @init_occgrid_state() local_unnamed_addr #3

declare dso_local void @xmit_pipe_init(...) local_unnamed_addr #3

declare dso_local void @recv_pipe_init(...) local_unnamed_addr #3

declare dso_local i32 @cv_toolset_init(...) local_unnamed_addr #3

; Function Attrs: nounwind
declare dso_local void (i32)* @signal(i32, void (i32)*) local_unnamed_addr #8

; Function Attrs: nounwind
declare dso_local i32 @getopt(i32, i8**, i8*) local_unnamed_addr #8

; Function Attrs: inlinehint nounwind readonly uwtable
define available_externally dso_local i32 @atoi(i8* nonnull %__nptr) local_unnamed_addr #9 {
entry:
  %call = tail call i64 @strtol(i8* nocapture nonnull %__nptr, i8** null, i32 10) #11
  %conv = trunc i64 %call to i32
  ret i32 %conv
}

; Function Attrs: nounwind
declare dso_local i32 @socket(i32, i32, i32) local_unnamed_addr #8

; Function Attrs: nounwind
declare dso_local i32 @inet_addr(i8*) local_unnamed_addr #8

declare dso_local i32 @connect(i32, %struct.sockaddr*, i32) local_unnamed_addr #3

declare dso_local i32 @sleep(i32) local_unnamed_addr #3

; Function Attrs: nofree nounwind
declare dso_local i32 @gettimeofday(%struct.timeval* nocapture, %struct.timezone* nocapture) local_unnamed_addr #1

; Function Attrs: nofree nounwind uwtable
define dso_local void @dump_final_run_statistics() local_unnamed_addr #0 {
entry:
  %0 = load i32, i32* @odo_count, align 4, !tbaa !2
  %1 = load i32, i32* @lidar_count, align 4, !tbaa !2
  %2 = load i32, i32* @lmap_count, align 4, !tbaa !2
  %3 = load i32, i32* @xmit_count, align 4, !tbaa !2
  %4 = load i32, i32* @recv_count, align 4, !tbaa !2
  %5 = load i32, i32* @car_send_count, align 4, !tbaa !2
  %6 = load i32, i32* @cv_count, align 4, !tbaa !2
  %call = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([90 x i8], [90 x i8]* @.str.79, i64 0, i64 0), i32 %0, i32 %1, i32 %2, i32 %3, i32 %4, i32 %5, i32 %6)
  %call1 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([59 x i8], [59 x i8]* @.str.80, i64 0, i64 0), i32 100, i32 100, double 2.000000e+00, i32 100)
  %call2 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([18 x i8], [18 x i8]* @.str.81, i64 0, i64 0))
  %call3 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([16 x i8], [16 x i8]* @.str.82, i64 0, i64 0))
  %call4 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @.str.83, i64 0, i64 0))
  %call5 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([20 x i8], [20 x i8]* @.str.84, i64 0, i64 0))
  %putchar = tail call i32 @putchar(i32 10)
  %call7 = tail call i32 @gettimeofday(%struct.timeval* nonnull @stop_prog, %struct.timezone* null) #11
  %7 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_prog, i64 0, i32 0), align 8, !tbaa !19
  %8 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_prog, i64 0, i32 0), align 8, !tbaa !19
  %sub = sub nsw i64 %7, %8
  %mul = mul i64 %sub, 1000000
  %9 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @stop_prog, i64 0, i32 1), align 8, !tbaa !23
  %10 = load i64, i64* getelementptr inbounds (%struct.timeval, %struct.timeval* @start_prog, i64 0, i32 1), align 8, !tbaa !23
  %sub8 = sub i64 %9, %10
  %add = add i64 %sub8, %mul
  %11 = load i64, i64* @proc_rdbag_sec, align 8, !tbaa !22
  %mul9 = mul i64 %11, 1000000
  %12 = load i64, i64* @proc_rdbag_usec, align 8, !tbaa !22
  %add10 = add i64 %mul9, %12
  %13 = load i64, i64* @proc_odo_sec, align 8, !tbaa !22
  %mul11 = mul i64 %13, 1000000
  %14 = load i64, i64* @proc_odo_usec, align 8, !tbaa !22
  %add12 = add i64 %mul11, %14
  %15 = load i64, i64* @proc_lidar_sec, align 8, !tbaa !22
  %mul13 = mul i64 %15, 1000000
  %16 = load i64, i64* @proc_lidar_usec, align 8, !tbaa !22
  %add14 = add i64 %mul13, %16
  %17 = load i64, i64* @proc_data_sec, align 8, !tbaa !22
  %mul15 = mul i64 %17, 1000000
  %18 = load i64, i64* @proc_data_usec, align 8, !tbaa !22
  %add16 = add i64 %mul15, %18
  %19 = load i64, i64* @proc_cv_sec, align 8, !tbaa !22
  %mul17 = mul i64 %19, 1000000
  %20 = load i64, i64* @proc_cv_usec, align 8, !tbaa !22
  %add18 = add i64 %mul17, %20
  %call19 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([40 x i8], [40 x i8]* @.str.86, i64 0, i64 0), i64 %add)
  %call20 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([42 x i8], [42 x i8]* @.str.87, i64 0, i64 0), i64 %add10)
  %call21 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([42 x i8], [42 x i8]* @.str.88, i64 0, i64 0), i64 %add12)
  %call22 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([42 x i8], [42 x i8]* @.str.89, i64 0, i64 0), i64 %add14)
  %call23 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([44 x i8], [44 x i8]* @.str.90, i64 0, i64 0), i64 %add16)
  %call24 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([42 x i8], [42 x i8]* @.str.91, i64 0, i64 0), i64 %add18)
  %puts = tail call i32 @puts(i8* getelementptr inbounds ([52 x i8], [52 x i8]* @str.129, i64 0, i64 0))
  %puts33 = tail call i32 @puts(i8* getelementptr inbounds ([22 x i8], [22 x i8]* @str.130, i64 0, i64 0))
  ret void
}

; Function Attrs: nofree nounwind
declare i32 @puts(i8* nocapture readonly) local_unnamed_addr #10

; Function Attrs: argmemonly nounwind
declare void @llvm.memcpy.p0i8.p0i8.i64(i8* nocapture writeonly, i8* nocapture readonly, i64, i1 immarg) #6

; Function Attrs: nofree nounwind
declare i32 @putchar(i32) local_unnamed_addr #10

attributes #0 = { nofree nounwind uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nofree nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #2 = { noreturn nounwind uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #3 = { "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #4 = { noreturn nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #5 = { nounwind uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #6 = { argmemonly nounwind }
attributes #7 = { nofree "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #8 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #9 = { inlinehint nounwind readonly uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #10 = { nofree nounwind }
attributes #11 = { nounwind }
attributes #12 = { noreturn nounwind }
attributes #13 = { nounwind readonly }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 9.0.0 (https://gitlab.engr.illinois.edu/llvm/hpvm.git eada7e7b77a4913538b6fc74d0e4578dabaa4d6f)"}
!2 = !{!3, !3, i64 0}
!3 = !{!"int", !4, i64 0}
!4 = !{!"omnipotent char", !5, i64 0}
!5 = !{!"Simple C/C++ TBAA"}
!6 = !{!7, !7, i64 0}
!7 = !{!"any pointer", !4, i64 0}
!8 = !{!4, !4, i64 0}
!9 = !{!10, !10, i64 0}
!10 = !{!"float", !4, i64 0}
!11 = !{!12, !3, i64 12}
!12 = !{!"lidar_inputs_struct", !4, i64 0, !3, i64 12, !4, i64 16}
!13 = !{!14, !15, i64 0}
!14 = !{!"sockaddr_in", !15, i64 0, !15, i64 2, !16, i64 4, !4, i64 8}
!15 = !{!"short", !4, i64 0}
!16 = !{!"in_addr", !3, i64 0}
!17 = !{!14, !3, i64 4}
!18 = !{!14, !15, i64 2}
!19 = !{!20, !21, i64 0}
!20 = !{!"timeval", !21, i64 0, !21, i64 8}
!21 = !{!"long", !4, i64 0}
!22 = !{!21, !21, i64 0}
!23 = !{!20, !21, i64 8}
