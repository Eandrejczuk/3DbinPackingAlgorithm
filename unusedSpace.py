import time
import cplex
import math
import numpy as np
from docplex.mp.model import Model

def defineLP(parcels, container, LPfile, solutionFile, coordinateFile, timeLimit):
    nbParcels = len(parcels)
    try:
        cargo_model = Model(name='CargoLoadUp')

        #variables
        #the variable to know if the parcel entered to the container
        p =  cargo_model.binary_var_list(nbParcels)

        x = cargo_model.integer_var_list(nbParcels, 0, container.length)
        y = cargo_model.integer_var_list(nbParcels, 0, container.width)
        z = cargo_model.integer_var_list(nbParcels, 0, container.height)
        #define x', y', z'
        xp = cargo_model.integer_var_list(nbParcels, 0, container.length)
        yp = cargo_model.integer_var_list(nbParcels, 0, container.width)
        zp = cargo_model.integer_var_list(nbParcels, 0, container.height)

        #variables for overlapping
        idx = [(i,j) for i in range(nbParcels) for j in range(nbParcels)]
        xb = cargo_model.binary_var_dict(idx)
        yb = cargo_model.binary_var_dict(idx)
        zb = cargo_model.binary_var_dict(idx)

        #variables for floating parcels
        absValue = cargo_model.integer_var_dict(idx, 0, container.height)
        goodHeight = cargo_model.binary_var_dict(idx)
        overlapXY = cargo_model.binary_var_dict(idx)
        stack = cargo_model.binary_var_dict(idx)
        ground = cargo_model.binary_var_list(nbParcels)

        #rotations variables
        r11 = cargo_model.binary_var_list(nbParcels)
        r12 = cargo_model.binary_var_list(nbParcels)
        r13 = cargo_model.binary_var_list(nbParcels)
        r21 = cargo_model.binary_var_list(nbParcels)
        r22 = cargo_model.binary_var_list(nbParcels)
        r23 = cargo_model.binary_var_list(nbParcels)
        r31 = cargo_model.binary_var_list(nbParcels)
        r32 = cargo_model.binary_var_list(nbParcels)
        r33 = cargo_model.binary_var_list(nbParcels)

        ####################### OBJECTIVE FUNCTION #########################

        objectiveFunction = cargo_model.linear_expr()
        for i in range(nbParcels):
            objectiveFunction.add(parcels[i].calculate_volume() * p[i])
        cargo_model.maximize(objectiveFunction)

        ####################### CONSTRAINTS ################################

        # Geometric Constraints

        #parcels do not exceed volume
        volumeConstraint = cargo_model.linear_expr()
        for i in range(nbParcels):
            volumeConstraint.add(parcels[i].calculate_volume() * p[i])
        volumeConstraint.add(-container.calculate_volume())
        cargo_model.add_constraint(volumeConstraint <= 0)

        #weight constraint
        weightConstraint = cargo_model.linear_expr()
        for i in range(nbParcels):
            weightConstraint.add(parcels[i].weight * p[i])
        weightConstraint.add(-container.weight)
        cargo_model.add_constraint(weightConstraint <= 0)

        #Parcels do not exceed dimensions of the container

        for i in range(nbParcels):
            #length
            containerDimensionsConstraint = cargo_model.linear_expr()
            containerDimensionsConstraint.add(xp[i])
            containerDimensionsConstraint.add(-container.length * p[i])
            cargo_model.add_constraint(containerDimensionsConstraint <= 0)
            #width
            containerDimensionsConstraint = cargo_model.linear_expr()
            containerDimensionsConstraint.add(yp[i])
            containerDimensionsConstraint.add(-container.width * p[i])
            cargo_model.add_constraint(containerDimensionsConstraint <= 0)
            #height
            containerDimensionsConstraint = cargo_model.linear_expr()
            containerDimensionsConstraint.add(zp[i])
            containerDimensionsConstraint.add(-container.height * p[i])
            cargo_model.add_constraint(containerDimensionsConstraint <= 0)

        #Orientation Constraints
        for i in range(nbParcels):
            cx = cargo_model.linear_expr()
            cx.add(xp[i])
            cx.add(-x[i])
            cx.add(-parcels[i].length*r11[i])
            cx.add(-parcels[i].width * r12[i])
            cx.add(-parcels[i].height * r13[i])
            cargo_model.add_constraint(cx == 0)

            cy = cargo_model.linear_expr()
            cy.add(yp[i])
            cy.add(-y[i])
            cy.add(-parcels[i].length*r21[i])
            cy.add(-parcels[i].width * r22[i])
            cy.add(-parcels[i].height * r23[i])
            cargo_model.add_constraint(cy == 0)

            cz = cargo_model.linear_expr()
            cz.add(zp[i])
            cz.add(-z[i])
            cz.add(-parcels[i].length*r31[i])
            cz.add(-parcels[i].width * r32[i])
            cz.add(-parcels[i].height * r33[i])
            cargo_model.add_constraint(cz == 0)

            c11 = cargo_model.linear_expr()
            c11.add(r11[i])
            c11.add(r12[i])
            c11.add(r13[i])
            cargo_model.add_constraint(c11 == 1)

            c21 = cargo_model.linear_expr()
            c21.add(r21[i])
            c21.add(r22[i])
            c21.add(r23[i])
            cargo_model.add_constraint(c21 == 1)

            c31 = cargo_model.linear_expr()
            c31.add(r31[i])
            c31.add(r32[i])
            c31.add(r33[i])
            cargo_model.add_constraint(c31 == 1)

            cc1 = cargo_model.linear_expr()
            cc1.add(r11[i])
            cc1.add(r21[i])
            cc1.add(r31[i])
            cargo_model.add_constraint(cc1 == 1)

            cc2 = cargo_model.linear_expr()
            cc2.add(r12[i])
            cc2.add(r22[i])
            cc2.add(r32[i])
            cargo_model.add_constraint(cc2 == 1)

            cc3 = cargo_model.linear_expr()
            cc3.add(r13[i])
            cc3.add(r23[i])
            cc3.add(r33[i])
            cargo_model.add_constraint(cc3 == 1)

            #Specific allowed rotations
            rotationLengthConstraint = cargo_model.linear_expr()
            rotationLengthConstraint.add(r31[i])
            rotationLengthConstraint.add(-parcels[i].rotationLength)
            cargo_model.add_constraint(rotationLengthConstraint <= 0)

            rotationWidthConstraint = cargo_model.linear_expr()
            rotationWidthConstraint.add(r32[i])
            rotationWidthConstraint.add(-parcels[i].rotationWidth)
            cargo_model.add_constraint(rotationWidthConstraint <= 0)

            rotationHeightConstaint = cargo_model.linear_expr()
            rotationHeightConstaint.add(r33[i])
            rotationHeightConstaint.add(-parcels[i].rotationHeight)
            cargo_model.add_constraint(rotationHeightConstaint <= 0)

        #Overlap Constraints
        for i in range(nbParcels):
            for k in range(nbParcels):
                # xp[k] - x[i] + xb[i][k].L <= L
                xbCons1 = cargo_model.linear_expr()
                xbCons1.add(xp[k])
                xbCons1.add(-x[i])
                xbCons1.add(container.length*xb[i,k])
                xbCons1.add(-container.length)
                cargo_model.add_constraint(xbCons1 <= 0)

                # xi-xp[k]-xb[i][k].L<=-1
                xbCons2 = cargo_model.linear_expr()
                xbCons2.add(x[i])
                xbCons2.add(-xp[k])
                xbCons2.add(-container.length*xb[i,k])
                cargo_model.add_constraint(xbCons2 <= -1)

                # yp[k] - y[i] + yb[i][k].W <= W
                ybCons1 = cargo_model.linear_expr()
                ybCons1.add(yp[k])
                ybCons1.add(-y[i])
                ybCons1.add(container.width*yb[i,k])
                ybCons1.add(-container.width)
                cargo_model.add_constraint(ybCons1 <= 0)

                # yi-yp[k]-yb[i][k].W<=-1
                ybCons2 = cargo_model.linear_expr()
                ybCons2.add(y[i])
                ybCons2.add(-yp[k])
                ybCons2.add(-container.length*yb[i,k])
                cargo_model.add_constraint(ybCons2 <= -1)

                # zp[k]-z[i]+zb[i][k].H<=H
                zbCons1 = cargo_model.linear_expr()
                zbCons1.add(zp[k])
                zbCons1.add(-z[i])
                zbCons1.add(container.height * zb[i, k])
                zbCons1.add(-container.height)
                cargo_model.add_constraint(zbCons1 <= 0)

        # xa+xb+ya+yb+za+zb>=pij+pkj-1
        for i in range(nbParcels):
            for k in range(i):
                noOverlap = cargo_model.linear_expr()
                noOverlap.add(xb[i, k])
                noOverlap.add(xb[k, i])
                noOverlap.add(yb[i, k])
                noOverlap.add(yb[k, i])
                noOverlap.add(zb[i, k])
                noOverlap.add(zb[k, i])
                noOverlap.add(-p[i])
                noOverlap.add(-p[k])
                cargo_model.add_constraint(noOverlap >= -1)

        #floating parcels
        for i in range(nbParcels):
            groundConstraint = cargo_model.linear_expr()
            groundConstraint.add(z[i])
            groundConstraint.add(container.height*ground[i])
            groundConstraint.add(-container.height)
            cargo_model.add_constraint(groundConstraint <= 0)

        for i in range(nbParcels):
            for k in range(nbParcels):
                # zpk-zi<=aik
                gdHt1 = cargo_model.linear_expr()
                gdHt1.add(zp[k])
                gdHt1.add(-z[i])
                gdHt1.add(-absValue[i, k])
                cargo_model.add_constraint(gdHt1 <= 0)

                #zi - zpk <= aik
                gdHt2 = cargo_model.linear_expr()
                gdHt2.add(z[i])
                gdHt2.add(-zp[k])
                gdHt2.add(-absValue[i, k])
                cargo_model.add_constraint(gdHt2 <= 0)

        #aik <= ghik * H
        for i in range(nbParcels):
            for k in range(nbParcels):
                gdHt3 = cargo_model.linear_expr()
                gdHt3.add(absValue[i, k])
                gdHt3.add(-container.height*goodHeight[i, k])
                cargo_model.add_constraint(gdHt3 <= 0)

        temp = cargo_model.binary_var_dict(idx)
        #zi-z'k+aik+2Hyik<=2H
        for i in range(nbParcels):
            for k in range(nbParcels):
                gdHt4 = cargo_model.linear_expr()
                gdHt4.add(-zp[k])
                gdHt4.add(z[i])
                gdHt4.add(absValue[i, k])
                gdHt4.add(2*container.height*temp[i, k])
                cargo_model.add_constraint(gdHt4 <= 2*container.height)

                #z'k-zi+aik-2Hyik<=0
                gdHt5 = cargo_model.linear_expr()
                gdHt5.add(zp[k])
                gdHt5.add(-z[i])
                gdHt5.add(absValue[i, k])
                gdHt5.add(-2*container.height*temp[i, k])
                cargo_model.add_constraint(gdHt5 <= 0)

        #hik<=aik
        for i in range(nbParcels):
            for k in range(nbParcels):
                gdHt6 = cargo_model.linear_expr()
                gdHt6.add(goodHeight[i, k])
                gdHt6.add(-absValue[i, k])
                cargo_model.add_constraint(gdHt6 <= 0)

        for i in range(nbParcels):
            for k in range(nbParcels):
                # xa[i][k]+xb[i][k]+ya[i][k]+yb[i][k]<=2oik
                overXY = cargo_model.linear_expr()
                overXY.add(xb[i, k])
                overXY.add(xb[k, i])
                overXY.add(yb[i, k])
                overXY.add(yb[k, i])
                overXY.add(-2*overlapXY[i,k])
                cargo_model.add_constraint(overXY <= 0)

                #xa[i][k] + xb[i][k] + ya[i][k] + yb[i][k] >= oik
                overXY2 = cargo_model.linear_expr()
                overXY2.add(xb[i, k])
                overXY2.add(xb[k, i])
                overXY2.add(yb[i, k])
                overXY2.add(yb[k, i])
                overXY2.add(-overlapXY[i,k])
                cargo_model.add_constraint(overXY2 >= 0)

        for i in range(nbParcels):
            for k in range(nbParcels):
                #hik+oik<=2(1-sik)
                stackConstraint = cargo_model.linear_expr()
                stackConstraint.add(goodHeight[i, k])
                stackConstraint.add(overlapXY[i, k])
                stackConstraint.add(2*stack[i, k])
                cargo_model.add_constraint(stackConstraint <= 2)

                #hik+oik>=(1-sik)
                stackConstraint2 = cargo_model.linear_expr()
                stackConstraint2.add(goodHeight[i, k])
                stackConstraint2.add(overlapXY[i, k])
                stackConstraint2.add(stack[i, k])
                cargo_model.add_constraint(stackConstraint2 >= 1)

        for i in range(nbParcels):
            for k in range(nbParcels):
                # p[i]-p[k]<=1-s[i][k]
                sameBin = cargo_model.linear_expr()
                sameBin.add(p[i])
                sameBin.add(-p[k])
                sameBin.add(stack[i,k])
                cargo_model.add_constraint(sameBin <= 1)

                #p[k] - p[i] <= 1 - s[i][k] - is this necessary??
                #sameBin2 = cargo_model.linear_expr()
                #sameBin2.add(-p[i])
                #sameBin2.add(p[k])
                #sameBin2.add(stack[i,k])
                #cargo_model.add_constraint(sameBin2 <= 1)

        # Vertical stability
        eta1 = cargo_model.binary_var_dict(idx)
        eta2 = cargo_model.binary_var_dict(idx)
        eta3 = cargo_model.binary_var_dict(idx)
        eta4 = cargo_model.binary_var_dict(idx)

        for i in range(nbParcels):
            for k in range(nbParcels):
                defEta1 = cargo_model.linear_expr()
                defEta1.add(x[k])
                defEta1.add(-x[i])
                defEta1.add(-container.length*eta1[i, k])
                cargo_model.add_constraint(defEta1 <= 0)

                defEta2 = cargo_model.linear_expr()
                defEta2.add(y[k])
                defEta2.add(-y[i])
                defEta2.add(-container.width*eta2[i, k])
                cargo_model.add_constraint(defEta2 <= 0)

                defEta3 = cargo_model.linear_expr()
                defEta3.add(-xp[k])
                defEta3.add(xp[i])
                defEta3.add(-container.length*eta3[i, k])
                cargo_model.add_constraint(defEta3 <= 0)

                defEta4 = cargo_model.linear_expr()
                defEta4.add(-yp[k])
                defEta4.add(yp[i])
                defEta4.add(-container.width*eta4[i, k])
                cargo_model.add_constraint(defEta4 <= 0)

        beta1 = cargo_model.binary_var_dict(idx)
        beta2 = cargo_model.binary_var_dict(idx)
        beta3 = cargo_model.binary_var_dict(idx)
        beta4 = cargo_model.binary_var_dict(idx)

        for i in range(nbParcels):
            for k in range(nbParcels):
                #beta<=s_ik
                defBeta1 = cargo_model.linear_expr()
                defBeta1.add(beta1[i, k])
                defBeta1.add(-stack[i,k])
                cargo_model.add_constraint(defBeta1 <= 0)

                defBeta2 = cargo_model.linear_expr()
                defBeta2.add(beta2[i, k])
                defBeta2.add(-stack[i,k])
                cargo_model.add_constraint(defBeta2 <= 0)

                defBeta3 = cargo_model.linear_expr()
                defBeta3.add(beta3[i, k])
                defBeta3.add(-stack[i,k])
                cargo_model.add_constraint(defBeta3 <= 0)

                defBeta4 = cargo_model.linear_expr()
                defBeta4.add(beta4[i, k])
                defBeta4.add(-stack[i,k])
                cargo_model.add_constraint(defBeta4 <= 0)

        for i in range(nbParcels):
            for k in range(nbParcels):
                defBeta1Bis = cargo_model.linear_expr()
                defBeta1Bis.add(eta1[i, k])
                defBeta1Bis.add(eta2[i, k])
                defBeta1Bis.add(2*beta1[i, k])
                cargo_model.add_constraint(defBeta1Bis <= 2)

                defBeta2Bis = cargo_model.linear_expr()
                defBeta2Bis.add(eta3[i, k])
                defBeta2Bis.add(eta2[i, k])
                defBeta2Bis.add(2*beta2[i, k])
                cargo_model.add_constraint(defBeta2Bis <= 2)

                defBeta3Bis = cargo_model.linear_expr()
                defBeta3Bis.add(eta3[i, k])
                defBeta3Bis.add(eta4[i, k])
                defBeta3Bis.add(2*beta3[i, k])
                cargo_model.add_constraint(defBeta3Bis <= 2)

                defBeta4Bis = cargo_model.linear_expr()
                defBeta4Bis.add(eta1[i, k])
                defBeta4Bis.add(eta4[i, k])
                defBeta4Bis.add(2*beta4[i, k])
                cargo_model.add_constraint(defBeta4Bis <= 2)

        # stability constraints
        for i in range(nbParcels):
            verticalStability = cargo_model.linear_expr()
            for k in range(nbParcels):
                verticalStability.add(beta1[i,k])
                verticalStability.add(beta2[i, k])
                verticalStability.add(beta3[i, k])
                verticalStability.add(beta4[i, k])
            verticalStability.add(3*ground[i])
            cargo_model.add(verticalStability >= 3)

        #cargo_model.print_information()

        cargo_model.export(LPfile)

        #LPsolution = solveLP(LPfile, solutionFile, timeLimit)

        #print(cargo_model.solve().get_values(p))

        if cargo_model.solve():
            volumeParcels = 0
            output = open(coordinateFile, 'w')
            output.write('parcel_ID, Container_ID, Pos_L, Pos_W, Pos_H, Size_L, Size_W, Size_H\n' )
            for i in range(nbParcels):
                print(p[i].solution_value)
                if p[i].solution_value != 0:
                    volumeParcels += parcels[i].calculate_volume()
                    output.write(str(parcels[i].id) + ",1," + str(x[i].solution_value)
                                 + "," + str(y[i].solution_value) + ","
                                 + str(z[i].solution_value)+ ",")
                    l = xp[i].solution_value - x[i].solution_value
                    w = yp[i].solution_value - y[i].solution_value
                    h = zp[i].solution_value - z[i].solution_value
                    output.write(str(l) + ","+ str(w) + ","+ str(h) + '\n')

            print('Volume utilization= %.8f%%' % (100*volumeParcels/container.calculate_volume()))
        else:
            print("Cannot solve")

        return cargo_model

    except cplex.exceptions.CplexError:
        print('Cannot solve')


def solveLP(LpFile, solutionFile, timeLimit):
    try:
        m = cplex.Cplex(LpFile)

        m.parameters.timelimit.set(timeLimit)
        start_time = time.time()
        m.solve()
        final_time = time.time() - start_time
        m.solution.write(solutionFile)

        print("solving time: ", final_time)
        print("Status: ", m.solution.status[m.solution.get_status()])

        print(m.solution.get_values())
        return m.solution.get_values()

    except cplex.exceptions.CplexError:
        print('Cannot solve')