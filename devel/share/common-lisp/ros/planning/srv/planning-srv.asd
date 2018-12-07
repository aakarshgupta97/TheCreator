
(cl:in-package :asdf)

(defsystem "planning-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BuildStructure" :depends-on ("_package_BuildStructure"))
    (:file "_package_BuildStructure" :depends-on ("_package"))
    (:file "ModelGenerator" :depends-on ("_package_ModelGenerator"))
    (:file "_package_ModelGenerator" :depends-on ("_package"))
  ))