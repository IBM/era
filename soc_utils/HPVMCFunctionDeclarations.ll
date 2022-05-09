; ModuleID = 'HPVMCFunctionDeclarations.cpp'
source_filename = "HPVMCFunctionDeclarations.cpp"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

@.str = private unnamed_addr constant [3 x i8] c"%p\00", align 1

; Function Attrs: nofree nounwind uwtable
define dso_local void @DummyUserFunction() local_unnamed_addr #0 {
entry:
  %call = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i32)* @__hpvm__hint to i8*))
  %call1 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i32, ...)* @__hpvm__createNodeND to i8*))
  %call2 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i32, ...)* @__hpvm__return to i8*))
  %call3 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i32, ...)* @__hpvm__attributes to i8*))
  %call4 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void ()* @__hpvm__init to i8*))
  %call5 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void ()* @__hpvm__cleanup to i8*))
  %call6 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*, i32, i32, i32)* @__hpvm__bindIn to i8*))
  %call7 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*, i32, i32, i32)* @__hpvm__bindOut to i8*))
  %call8 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i8*, i8*, i32, i32, i32, i32)* @__hpvm__edge to i8*))
  %call9 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i32)* @__hpvm__hint to i8*))
  %call10 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i32, ...)* @__hpvm__createNodeND to i8*))
  %call11 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i32, ...)* @__hpvm__return to i8*))
  %call12 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i32, ...)* @__hpvm__attributes to i8*))
  %call13 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void ()* @__hpvm__init to i8*))
  %call14 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void ()* @__hpvm__cleanup to i8*))
  %call15 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*, i32, i32, i32)* @__hpvm__bindIn to i8*))
  %call16 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*, i32, i32, i32)* @__hpvm__bindOut to i8*))
  %call17 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i8*, i8*, i32, i32, i32, i32)* @__hpvm__edge to i8*))
  %call18 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*, i8*)* @__hpvm__push to i8*))
  %call19 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i8*)* @__hpvm__pop to i8*))
  %call20 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i32, ...)* @__hpvm__launch to i8*))
  %call21 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*)* @__hpvm__wait to i8*))
  %call22 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* ()* @__hpvm__getNode to i8*))
  %call23 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i8*)* @__hpvm__getParentNode to i8*))
  %call24 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void ()* @__hpvm__barrier to i8*))
  %call25 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i8* (i64)* @__hpvm__malloc to i8*))
  %call26 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i64 (i8*)* @__hpvm__getNodeInstanceID_x to i8*))
  %call27 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i64 (i8*)* @__hpvm__getNodeInstanceID_y to i8*))
  %call28 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i64 (i8*)* @__hpvm__getNodeInstanceID_z to i8*))
  %call29 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i64 (i8*)* @__hpvm__getNumNodeInstances_x to i8*))
  %call30 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i64 (i8*)* @__hpvm__getNumNodeInstances_y to i8*))
  %call31 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i64 (i8*)* @__hpvm__getNumNodeInstances_z to i8*))
  %call32 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_add to i8*))
  %call33 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_sub to i8*))
  %call34 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_xchg to i8*))
  %call35 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*)* @__hpvm__atomic_inc to i8*))
  %call36 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*)* @__hpvm__atomic_dec to i8*))
  %call37 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_min to i8*))
  %call38 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_max to i8*))
  %call39 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_and to i8*))
  %call40 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_or to i8*))
  %call41 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (i32 (i32*, i32)* @__hpvm__atomic_xor to i8*))
  %call42 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*, i64)* @llvm_hpvm_track_mem to i8*))
  %call43 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*)* @llvm_hpvm_untrack_mem to i8*))
  %call44 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i8* bitcast (void (i8*, i64)* @llvm_hpvm_request_mem to i8*))
  ret void
}

; Function Attrs: nofree nounwind
declare dso_local i32 @printf(i8* nocapture readonly, ...) local_unnamed_addr #1

; Function Attrs: nounwind
declare dso_local void @__hpvm__hint(i32) #2

; Function Attrs: nounwind
declare dso_local i8* @__hpvm__createNodeND(i32, ...) #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__return(i32, ...) #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__attributes(i32, ...) #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__init() #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__cleanup() #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__bindIn(i8*, i32, i32, i32) #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__bindOut(i8*, i32, i32, i32) #2

; Function Attrs: nounwind
declare dso_local i8* @__hpvm__edge(i8*, i8*, i32, i32, i32, i32) #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__push(i8*, i8*) #2

; Function Attrs: nounwind
declare dso_local i8* @__hpvm__pop(i8*) #2

; Function Attrs: nounwind
declare dso_local i8* @__hpvm__launch(i32, ...) #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__wait(i8*) #2

; Function Attrs: nounwind
declare dso_local i8* @__hpvm__getNode() #2

; Function Attrs: nounwind
declare dso_local i8* @__hpvm__getParentNode(i8*) #2

; Function Attrs: nounwind
declare dso_local void @__hpvm__barrier() #2

; Function Attrs: nounwind
declare dso_local i8* @__hpvm__malloc(i64) #2

; Function Attrs: nounwind
declare dso_local i64 @__hpvm__getNodeInstanceID_x(i8*) #2

; Function Attrs: nounwind
declare dso_local i64 @__hpvm__getNodeInstanceID_y(i8*) #2

; Function Attrs: nounwind
declare dso_local i64 @__hpvm__getNodeInstanceID_z(i8*) #2

; Function Attrs: nounwind
declare dso_local i64 @__hpvm__getNumNodeInstances_x(i8*) #2

; Function Attrs: nounwind
declare dso_local i64 @__hpvm__getNumNodeInstances_y(i8*) #2

; Function Attrs: nounwind
declare dso_local i64 @__hpvm__getNumNodeInstances_z(i8*) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_add(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_sub(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_xchg(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_inc(i32*) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_dec(i32*) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_min(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_max(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_and(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_or(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local i32 @__hpvm__atomic_xor(i32*, i32) #2

; Function Attrs: nounwind
declare dso_local void @llvm_hpvm_track_mem(i8*, i64) #2

; Function Attrs: nounwind
declare dso_local void @llvm_hpvm_untrack_mem(i8*) #2

; Function Attrs: nounwind
declare dso_local void @llvm_hpvm_request_mem(i8*, i64) #2

attributes #0 = { nofree nounwind uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nofree nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #2 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 9.0.0 (https://gitlab.engr.illinois.edu/llvm/hpvm.git adff8f039eac943c7b5efebf0f7765cac45b87b4)"}
