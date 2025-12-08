from NavOA import NavOA

if __name__ == '__main__':

    NavOA_planner = NavOA.init_from_yaml('dune_train_limo.yaml')
    NavOA_planner.train_dune()