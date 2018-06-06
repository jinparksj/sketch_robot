class TranslationPaper:
    def __init__(self):
        pass

    def translation_paper(self, x_old, y_old, x_new, y_new):
        tx = x_new - x_old
        ty = y_new - y_old

        return tx, ty

